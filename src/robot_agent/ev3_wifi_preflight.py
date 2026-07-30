"""Read-only EV3 Wi-Fi readiness inventory over an existing SSH link."""

import json
import subprocess
from typing import Any, Callable, Dict

from .ssh_policy import motion_free_ssh_options


MAX_OUTPUT_BYTES = 64 * 1024
DEFAULT_COMMAND_TIMEOUT_SECONDS = 30
MAX_COMMAND_TIMEOUT_SECONDS = 60
Runner = Callable[..., Any]


class EV3WiFiPreflightError(RuntimeError):
    """Base class for bounded Wi-Fi preflight failures."""


class EV3WiFiPreflightConfigurationError(EV3WiFiPreflightError):
    pass


class EV3WiFiPreflightTransportError(EV3WiFiPreflightError):
    pass


class EV3WiFiPreflightProtocolError(EV3WiFiPreflightError):
    pass


# This program is sent on stdin to a fixed ``python3 -`` command.  It remains
# Python 3.5 compatible because the current EV3 image is ev3dev-stretch.
REMOTE_PREFLIGHT_PROGRAM = r'''import glob
import json
import os
import platform
import shutil
import socket
import subprocess

MAX_CAPTURE = 8192


def read_file(path, limit=4096):
    try:
        with open(path, "rb") as handle:
            return handle.read(limit).decode("utf-8", "replace").strip()
    except (IOError, OSError):
        return ""


def capture(arguments):
    if not arguments or shutil.which(arguments[0]) is None:
        return {
            "available": False,
            "returncode": None,
            "output": "",
        }
    try:
        output = subprocess.check_output(
            arguments,
            stderr=subprocess.STDOUT,
            timeout=4,
        )
        return {
            "available": True,
            "returncode": 0,
            "output": output[:MAX_CAPTURE].decode(
                "utf-8",
                "replace",
            ).strip(),
        }
    except subprocess.CalledProcessError as error:
        output = error.output or b""
        return {
            "available": True,
            "returncode": int(error.returncode),
            "output": output[:MAX_CAPTURE].decode(
                "utf-8",
                "replace",
            ).strip(),
        }
    except subprocess.TimeoutExpired:
        return {
            "available": True,
            "returncode": None,
            "output": "command timed out",
        }
    except (IOError, OSError):
        return {
            "available": False,
            "returncode": None,
            "output": "",
        }


def os_release():
    result = {}
    for line in read_file("/etc/os-release").splitlines():
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        if key in ("ID", "VERSION_ID", "PRETTY_NAME"):
            result[key.lower()] = value.strip().strip('"')
    return result


def usb_devices():
    result = []
    for path in sorted(glob.glob("/sys/bus/usb/devices/*")):
        vendor = read_file(os.path.join(path, "idVendor"), 32)
        product_id = read_file(os.path.join(path, "idProduct"), 32)
        if not vendor or not product_id:
            continue
        result.append({
            "bus_path": os.path.basename(path),
            "id": "{}:{}".format(vendor, product_id),
            "manufacturer": read_file(
                os.path.join(path, "manufacturer"),
                256,
            ),
            "product": read_file(
                os.path.join(path, "product"),
                256,
            ),
        })
        if len(result) >= 32:
            break
    return result


def interfaces():
    result = []
    for path in sorted(glob.glob("/sys/class/net/*")):
        driver_path = os.path.join(path, "device", "driver")
        module_path = os.path.join(driver_path, "module")
        driver = ""
        driver_module = ""
        if os.path.exists(driver_path):
            driver = os.path.basename(os.path.realpath(driver_path))
        if os.path.exists(module_path):
            driver_module = os.path.basename(
                os.path.realpath(module_path)
            )
        result.append({
            "name": os.path.basename(path),
            "wireless": os.path.isdir(os.path.join(path, "wireless")),
            "operstate": read_file(os.path.join(path, "operstate"), 64),
            "address": read_file(os.path.join(path, "address"), 64),
            "driver": driver,
            "driver_module": driver_module,
        })
        if len(result) >= 64:
            break
    return result


kernel_release = platform.release()
network_interfaces = interfaces()
wireless_interfaces = [
    item["name"]
    for item in network_interfaces
    if item["wireless"]
]
ath9k_htc_interfaces = [
    item["name"]
    for item in network_interfaces
    if item["wireless"]
    and item["driver_module"] == "ath9k_htc"
]
firmware_candidates = [
    "/lib/firmware/ar9271.fw",
    "/lib/firmware/htc_9271.fw",
    "/lib/firmware/ath9k_htc/htc_9271-1.4.0.fw",
]
firmware_present = [
    path
    for path in firmware_candidates
    if os.path.isfile(path)
]
module_pattern = os.path.join(
    "/lib/modules",
    kernel_release,
    "**",
    "ath9k_htc.ko*",
)
module_files = sorted(
    glob.glob(module_pattern, recursive=True)
)[:16]
connman_technologies = capture(["connmanctl", "technologies"])
connman_services = capture(["connmanctl", "services"])
wifi_technology_present = (
    connman_technologies["available"]
    and connman_technologies["returncode"] == 0
    and any(
        line.strip() == "Type = wifi"
        for line in connman_technologies["output"].splitlines()
    )
)

result = {
    "schema_version": 1,
    "status": "observed",
    "effects": "read_only",
    "identity": {
        "hostname": socket.gethostname(),
        "machine_id": read_file("/etc/machine-id", 128),
    },
    "system": {
        "kernel_release": kernel_release,
        "os_release": os_release(),
    },
    "usb_devices": usb_devices(),
    "network": {
        "interfaces": network_interfaces,
        "wireless_interfaces": wireless_interfaces,
        "ath9k_htc_interfaces": ath9k_htc_interfaces,
        "addresses": capture(["ip", "-o", "address", "show"]),
        "routes": capture(["ip", "route", "show"]),
    },
    "ath9k_htc": {
        "module_loaded": os.path.isdir("/sys/module/ath9k_htc"),
        "module_files": module_files,
        "declared_firmware": capture([
            "modinfo",
            "-F",
            "firmware",
            "ath9k_htc",
        ]),
        "firmware_candidates": firmware_candidates,
        "firmware_present": firmware_present,
        "debian_package": capture([
            "dpkg-query",
            "-W",
            "-f=${Status} ${Version}\n",
            "firmware-atheros",
        ]),
    },
    "connman": {
        "available": shutil.which("connmanctl") is not None,
        "wifi_technology_present": wifi_technology_present,
        "technologies": connman_technologies,
        "services": connman_services,
    },
    "onboarding_ready": bool(ath9k_htc_interfaces)
        and shutil.which("connmanctl") is not None
        and wifi_technology_present,
}
print(json.dumps(
    result,
    ensure_ascii=True,
    separators=(",", ":"),
    sort_keys=True,
))
'''


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
        raise EV3WiFiPreflightConfigurationError(
            "SSH target is invalid"
        )
    return target


def _parse_result(raw: str) -> Dict[str, object]:
    if not isinstance(raw, str):
        raise EV3WiFiPreflightProtocolError(
            "EV3 preflight response was not text"
        )
    if len(raw.encode("utf-8")) > MAX_OUTPUT_BYTES:
        raise EV3WiFiPreflightProtocolError(
            "EV3 preflight response was too large"
        )
    def reject_constant(_value: str) -> None:
        raise ValueError("Non-finite JSON number")

    def strict_object(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError("Duplicate JSON key")
            result[key] = value
        return result

    try:
        result = json.loads(
            raw,
            object_pairs_hook=strict_object,
            parse_constant=reject_constant,
        )
    except (TypeError, ValueError):
        raise EV3WiFiPreflightProtocolError(
            "EV3 preflight returned invalid JSON"
        ) from None
    if not isinstance(result, dict):
        raise EV3WiFiPreflightProtocolError(
            "EV3 preflight response had an invalid schema"
        )
    schema_version = result.get("schema_version")
    network = result.get("network")
    connman = result.get("connman")
    if (
        isinstance(schema_version, bool)
        or not isinstance(schema_version, int)
        or schema_version != 1
        or result.get("status") != "observed"
        or result.get("effects") != "read_only"
        or not isinstance(result.get("identity"), dict)
        or not isinstance(network, dict)
        or not isinstance(result.get("ath9k_htc"), dict)
        or not isinstance(connman, dict)
        or not isinstance(result.get("onboarding_ready"), bool)
    ):
        raise EV3WiFiPreflightProtocolError(
            "EV3 preflight response had an invalid schema"
        )
    wireless_interfaces = network.get("wireless_interfaces")
    ath9k_htc_interfaces = network.get(
        "ath9k_htc_interfaces"
    )
    interfaces = network.get("interfaces")
    technologies = connman.get("technologies")
    if (
        not isinstance(interfaces, list)
        or not isinstance(wireless_interfaces, list)
        or not isinstance(ath9k_htc_interfaces, list)
        or not isinstance(connman.get("available"), bool)
        or not isinstance(
            connman.get("wifi_technology_present"),
            bool,
        )
        or not isinstance(technologies, dict)
        or not isinstance(technologies.get("available"), bool)
        or (
            technologies.get("returncode") is not None
            and (
                isinstance(technologies.get("returncode"), bool)
                or not isinstance(
                    technologies.get("returncode"),
                    int,
                )
            )
        )
    ):
        raise EV3WiFiPreflightProtocolError(
            "EV3 preflight readiness evidence was invalid"
        )
    if (
        any(
            not isinstance(name, str) or not name
            for name in wireless_interfaces
        )
        or any(
            not isinstance(name, str) or not name
            for name in ath9k_htc_interfaces
        )
    ):
        raise EV3WiFiPreflightProtocolError(
            "EV3 preflight interface evidence was invalid"
        )
    observed_wireless = []
    observed_ath9k_htc = []
    for interface in interfaces:
        if (
            not isinstance(interface, dict)
            or not isinstance(interface.get("name"), str)
            or not interface["name"]
            or not isinstance(interface.get("wireless"), bool)
            or not isinstance(interface.get("driver"), str)
            or not isinstance(
                interface.get("driver_module"),
                str,
            )
        ):
            raise EV3WiFiPreflightProtocolError(
                "EV3 preflight interface evidence was invalid"
            )
        if interface["wireless"]:
            observed_wireless.append(interface["name"])
            if interface["driver_module"] == "ath9k_htc":
                observed_ath9k_htc.append(interface["name"])
    if (
        wireless_interfaces != observed_wireless
        or ath9k_htc_interfaces != observed_ath9k_htc
    ):
        raise EV3WiFiPreflightProtocolError(
            "EV3 preflight interface evidence was inconsistent"
        )
    expected_ready = (
        bool(ath9k_htc_interfaces)
        and connman["available"]
        and connman["wifi_technology_present"]
        and technologies["available"]
        and technologies.get("returncode") == 0
    )
    if result["onboarding_ready"] is not expected_ready:
        raise EV3WiFiPreflightProtocolError(
            "EV3 preflight readiness was inconsistent"
        )
    return result


def run_ev3_wifi_preflight(
    target: str,
    runner: Runner = subprocess.run,
    connect_timeout_seconds: int = 3,
    command_timeout_seconds: int = DEFAULT_COMMAND_TIMEOUT_SECONDS,
) -> Dict[str, object]:
    """Inventory Wi-Fi readiness without changing EV3 network state."""

    validated_target = _validate_target(target)
    for name, value, maximum in (
        ("connect timeout", connect_timeout_seconds, 30),
        (
            "command timeout",
            command_timeout_seconds,
            MAX_COMMAND_TIMEOUT_SECONDS,
        ),
    ):
        if (
            isinstance(value, bool)
            or not isinstance(value, int)
            or not 1 <= value <= maximum
        ):
            raise EV3WiFiPreflightConfigurationError(
                "{} is invalid".format(name)
            )
    if not callable(runner):
        raise EV3WiFiPreflightConfigurationError(
            "SSH runner is invalid"
        )

    argv = (
        ["ssh", "-T"]
        + motion_free_ssh_options(connect_timeout_seconds)
        + [validated_target, "python3", "-"]
    )
    try:
        completed = runner(
            argv,
            input=REMOTE_PREFLIGHT_PROGRAM,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=command_timeout_seconds,
            check=False,
        )
    except subprocess.TimeoutExpired:
        raise EV3WiFiPreflightTransportError(
            "EV3 Wi-Fi preflight exceeded the {}-second "
            "command deadline".format(command_timeout_seconds)
        ) from None
    except OSError:
        raise EV3WiFiPreflightTransportError(
            "Could not start EV3 Wi-Fi preflight"
        ) from None

    returncode = getattr(completed, "returncode", None)
    stdout = getattr(completed, "stdout", None)
    stderr = getattr(completed, "stderr", None)
    if (
        isinstance(returncode, bool)
        or not isinstance(returncode, int)
        or not isinstance(stdout, str)
        or not isinstance(stderr, str)
    ):
        raise EV3WiFiPreflightProtocolError(
            "SSH runner returned an invalid result"
        )
    if returncode != 0:
        detail = " ".join(stderr.split())[:200]
        message = "EV3 Wi-Fi preflight failed with status {}".format(
            returncode
        )
        if detail:
            message += ": " + detail
        raise EV3WiFiPreflightTransportError(message)
    return _parse_result(stdout)
