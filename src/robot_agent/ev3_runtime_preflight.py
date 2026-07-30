"""Read-only verification of the code deployed to an EV3 runtime.

The remote command and both deployment manifests are fixed in this module.
Only the SSH target and the choice between those manifests are operator
inputs.  The preflight reads files and compares their SHA-256 digests; it
never starts a daemon or imports EV3 runtime code.
"""

import ast
import hashlib
import json
import os
import stat
import subprocess
from pathlib import Path
from typing import Any, Callable, Dict, Mapping, Tuple

from .ssh_policy import motion_free_ssh_options


REMOTE_PROJECT_ROOT = "/home/robot/robot-llm"
MAX_FILE_BYTES = 256 * 1024
MAX_OUTPUT_BYTES = 32 * 1024
DEFAULT_COMMAND_TIMEOUT_SECONDS = 20
MAX_COMMAND_TIMEOUT_SECONDS = 60

PERIPHERAL_MANIFEST = (
    "ev3/peripheral_daemon.py",
    "ev3/peripheral_protocol.py",
    "ev3/robot_hal.py",
    "ev3/robot_config.py",
    "ev3/emergency_stop.py",
    "config/ev3rstorm.json",
)
SUPERVISOR_ADDITIONS = (
    "ev3/supervisor_daemon.py",
    "ev3/supervisor_protocol.py",
    "ev3/supervisor.py",
    "ev3/supervisor_cli.py",
)
SUPERVISOR_MANIFEST = PERIPHERAL_MANIFEST + SUPERVISOR_ADDITIONS
PROFILE_MANIFESTS = {
    "peripheral": PERIPHERAL_MANIFEST,
    "supervisor": SUPERVISOR_MANIFEST,
}
ALL_MANIFEST_PATHS = SUPERVISOR_MANIFEST

Runner = Callable[..., Any]


class EV3RuntimePreflightError(RuntimeError):
    """Base class for bounded runtime deployment preflight failures."""

    code = "runtime_preflight_failed"


class EV3RuntimePreflightConfigurationError(EV3RuntimePreflightError):
    code = "invalid_configuration"


class EV3RuntimePreflightTransportError(EV3RuntimePreflightError):
    code = "transport_failed"


class EV3RuntimePreflightProtocolError(EV3RuntimePreflightError):
    code = "invalid_remote_reply"


class EV3RuntimeDeploymentMismatchError(EV3RuntimePreflightError):
    code = "deployment_mismatch"

    def __init__(self, code: str, message: str):
        self.code = code
        RuntimeError.__init__(self, message)


def _validate_manifests() -> None:
    if len(ALL_MANIFEST_PATHS) != len(set(ALL_MANIFEST_PATHS)):
        raise RuntimeError("Runtime deployment manifest contains duplicates")
    if not set(PERIPHERAL_MANIFEST).issubset(SUPERVISOR_MANIFEST):
        raise RuntimeError("Supervisor manifest omits peripheral files")
    for relative_path in ALL_MANIFEST_PATHS:
        path = Path(relative_path)
        if (
            not relative_path
            or path.is_absolute()
            or ".." in path.parts
            or "\\" in relative_path
        ):
            raise RuntimeError("Runtime deployment manifest is unsafe")


_validate_manifests()


# This fixed program is sent on stdin to ``python3 -``.  It deliberately
# avoids syntax newer than Python 3.5 for the ev3dev-stretch runtime.
_REMOTE_PREFLIGHT_TEMPLATE = r'''from __future__ import print_function

import hashlib
import json
import os
import stat

ROOT = "/home/robot/robot-llm"
MAX_FILE_BYTES = 262144
PATHS = __FIXED_MANIFEST_PATHS__


def failed(path, status):
    return {
        "path": path,
        "status": status,
        "size": None,
        "sha256": None,
    }


def ancestor_tokens(relative_path):
    current = ROOT
    tokens = []
    components = relative_path.split("/")[:-1]
    for component in [None] + components:
        if component is not None:
            current = os.path.join(current, component)
        try:
            evidence = os.lstat(current)
        except (IOError, OSError):
            return None
        if (
            stat.S_ISLNK(evidence.st_mode)
            or not stat.S_ISDIR(evidence.st_mode)
        ):
            return None
        tokens.append((evidence.st_dev, evidence.st_ino))
    return tokens


def inspect_file(relative_path):
    path = os.path.join(ROOT, relative_path)
    ancestors_before = ancestor_tokens(relative_path)
    if ancestors_before is None:
        return failed(relative_path, "unsafe_ancestor")
    try:
        before = os.lstat(path)
    except (IOError, OSError):
        return failed(relative_path, "missing")
    if stat.S_ISLNK(before.st_mode):
        return failed(relative_path, "symlink")
    if not stat.S_ISREG(before.st_mode):
        return failed(relative_path, "non_regular")
    if before.st_size > MAX_FILE_BYTES:
        return failed(relative_path, "oversized")

    flags = os.O_RDONLY
    flags |= getattr(os, "O_CLOEXEC", 0)
    flags |= getattr(os, "O_NOFOLLOW", 0)
    descriptor = None
    handle = None
    try:
        descriptor = os.open(path, flags)
        opened = os.fstat(descriptor)
        if (
            not stat.S_ISREG(opened.st_mode)
            or opened.st_dev != before.st_dev
            or opened.st_ino != before.st_ino
        ):
            return failed(relative_path, "changed")
        if opened.st_size > MAX_FILE_BYTES:
            return failed(relative_path, "oversized")
        handle = os.fdopen(descriptor, "rb")
        descriptor = None
        digest = hashlib.sha256()
        total = 0
        chunks = []
        while True:
            chunk = handle.read(16384)
            if not chunk:
                break
            total += len(chunk)
            if total > MAX_FILE_BYTES:
                return failed(relative_path, "oversized")
            digest.update(chunk)
            chunks.append(chunk)
        after = os.fstat(handle.fileno())
        if (
            after.st_dev != opened.st_dev
            or after.st_ino != opened.st_ino
            or after.st_size != total
        ):
            return failed(relative_path, "changed")
        try:
            path_after = os.lstat(path)
        except (IOError, OSError):
            return failed(relative_path, "changed")
        if stat.S_ISLNK(path_after.st_mode):
            return failed(relative_path, "symlink")
        if not stat.S_ISREG(path_after.st_mode):
            return failed(relative_path, "non_regular")
        if (
            path_after.st_dev != before.st_dev
            or path_after.st_ino != before.st_ino
        ):
            return failed(relative_path, "changed")
        ancestors_after = ancestor_tokens(relative_path)
        if ancestors_after is None:
            return failed(relative_path, "unsafe_ancestor")
        if ancestors_after != ancestors_before:
            return failed(relative_path, "changed")
        if relative_path.endswith(".py"):
            try:
                compile(
                    b"".join(chunks),
                    relative_path,
                    "exec",
                    dont_inherit=True,
                )
            except (
                SyntaxError,
                TypeError,
                ValueError,
                OverflowError,
            ):
                return failed(
                    relative_path,
                    "python_incompatible",
                )
        return {
            "path": relative_path,
            "status": "ok",
            "size": total,
            "sha256": digest.hexdigest(),
        }
    except (IOError, OSError):
        return failed(relative_path, "unreadable")
    finally:
        if handle is not None:
            handle.close()
        elif descriptor is not None:
            os.close(descriptor)


result = {
    "schema_version": 1,
    "effects": "read_only",
    "files": [inspect_file(path) for path in PATHS],
}
print(json.dumps(result, separators=(",", ":"), sort_keys=True))
'''

REMOTE_PREFLIGHT_PROGRAM = _REMOTE_PREFLIGHT_TEMPLATE.replace(
    "__FIXED_MANIFEST_PATHS__",
    json.dumps(list(ALL_MANIFEST_PATHS), separators=(",", ":")),
)


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
        raise EV3RuntimePreflightConfigurationError(
            "SSH target is invalid"
        )
    return target


def _validate_positive_int(
    name: str,
    value: object,
    maximum: int,
) -> int:
    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or not 1 <= value <= maximum
    ):
        raise EV3RuntimePreflightConfigurationError(
            "{} is invalid".format(name)
        )
    return value


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
        raise EV3RuntimePreflightProtocolError(
            "{} had an invalid schema".format(context)
        )
    return value


def _parse_remote_result(stdout: object) -> Dict[str, object]:
    if not isinstance(stdout, str):
        raise EV3RuntimePreflightProtocolError(
            "Remote preflight response was not text"
        )
    try:
        encoded = stdout.encode("utf-8")
    except UnicodeError:
        raise EV3RuntimePreflightProtocolError(
            "Remote preflight response was invalid"
        ) from None
    if not encoded or len(encoded) > MAX_OUTPUT_BYTES:
        raise EV3RuntimePreflightProtocolError(
            "Remote preflight response size was invalid"
        )
    try:
        parsed = json.loads(
            stdout,
            object_pairs_hook=_strict_object,
            parse_constant=_reject_constant,
        )
    except (TypeError, ValueError):
        raise EV3RuntimePreflightProtocolError(
            "Remote preflight returned invalid JSON"
        ) from None

    result = _exact_fields(
        parsed,
        ("schema_version", "effects", "files"),
        "Remote preflight response",
    )
    if (
        isinstance(result["schema_version"], bool)
        or result["schema_version"] != 1
        or result["effects"] != "read_only"
        or not isinstance(result["files"], list)
        or len(result["files"]) != len(ALL_MANIFEST_PATHS)
    ):
        raise EV3RuntimePreflightProtocolError(
            "Remote preflight response had an invalid schema"
        )

    validated_files = []
    expected_statuses = frozenset(
        (
            "ok",
            "missing",
            "symlink",
            "non_regular",
            "oversized",
            "changed",
            "unreadable",
            "unsafe_ancestor",
            "python_incompatible",
        )
    )
    for index, raw_entry in enumerate(result["files"]):
        entry = _exact_fields(
            raw_entry,
            ("path", "status", "size", "sha256"),
            "Remote file entry",
        )
        relative_path = ALL_MANIFEST_PATHS[index]
        status_value = entry["status"]
        if (
            entry["path"] != relative_path
            or not isinstance(status_value, str)
            or status_value not in expected_statuses
        ):
            raise EV3RuntimePreflightProtocolError(
                "Remote file entry was inconsistent"
            )
        if status_value == "ok":
            size = entry["size"]
            digest = entry["sha256"]
            if (
                isinstance(size, bool)
                or not isinstance(size, int)
                or not 0 <= size <= MAX_FILE_BYTES
                or not isinstance(digest, str)
                or len(digest) != 64
                or any(
                    character not in "0123456789abcdef"
                    for character in digest
                )
            ):
                raise EV3RuntimePreflightProtocolError(
                    "Remote file evidence was invalid"
                )
        elif entry["size"] is not None or entry["sha256"] is not None:
            raise EV3RuntimePreflightProtocolError(
                "Remote file failure evidence was invalid"
            )
        validated_files.append(dict(entry))
    return {
        "schema_version": 1,
        "effects": "read_only",
        "files": validated_files,
    }


def _hash_local_file(
    local_root: Path,
    relative_path: str,
) -> Tuple[int, str, bytes]:
    path = local_root / relative_path
    ancestors_before = _local_ancestor_tokens(
        local_root,
        relative_path,
    )
    try:
        before = os.lstat(str(path))
    except OSError:
        raise EV3RuntimePreflightConfigurationError(
            "Local manifest file is missing: {}".format(relative_path)
        ) from None
    if stat.S_ISLNK(before.st_mode) or not stat.S_ISREG(before.st_mode):
        raise EV3RuntimePreflightConfigurationError(
            "Local manifest file is not regular: {}".format(
                relative_path
            )
        )
    if before.st_size > MAX_FILE_BYTES:
        raise EV3RuntimePreflightConfigurationError(
            "Local manifest file is oversized: {}".format(relative_path)
        )

    flags = os.O_RDONLY
    flags |= getattr(os, "O_CLOEXEC", 0)
    flags |= getattr(os, "O_NOFOLLOW", 0)
    descriptor = None
    handle = None
    try:
        descriptor = os.open(str(path), flags)
        opened = os.fstat(descriptor)
        if (
            not stat.S_ISREG(opened.st_mode)
            or opened.st_dev != before.st_dev
            or opened.st_ino != before.st_ino
            or opened.st_size > MAX_FILE_BYTES
        ):
            raise EV3RuntimePreflightConfigurationError(
                "Local manifest file changed: {}".format(relative_path)
            )
        handle = os.fdopen(descriptor, "rb")
        descriptor = None
        digest = hashlib.sha256()
        total = 0
        chunks = []
        while True:
            chunk = handle.read(16384)
            if not chunk:
                break
            total += len(chunk)
            if total > MAX_FILE_BYTES:
                raise EV3RuntimePreflightConfigurationError(
                    "Local manifest file is oversized: {}".format(
                        relative_path
                    )
                )
            digest.update(chunk)
            chunks.append(chunk)
        after = os.fstat(handle.fileno())
        if (
            after.st_dev != opened.st_dev
            or after.st_ino != opened.st_ino
            or after.st_size != total
        ):
            raise EV3RuntimePreflightConfigurationError(
                "Local manifest file changed: {}".format(relative_path)
            )
        try:
            path_after = os.lstat(str(path))
        except OSError:
            raise EV3RuntimePreflightConfigurationError(
                "Local manifest file changed: {}".format(relative_path)
            ) from None
        if (
            stat.S_ISLNK(path_after.st_mode)
            or not stat.S_ISREG(path_after.st_mode)
            or path_after.st_dev != before.st_dev
            or path_after.st_ino != before.st_ino
        ):
            raise EV3RuntimePreflightConfigurationError(
                "Local manifest file changed: {}".format(relative_path)
            )
        ancestors_after = _local_ancestor_tokens(
            local_root,
            relative_path,
        )
        if ancestors_after != ancestors_before:
            raise EV3RuntimePreflightConfigurationError(
                "Local manifest ancestor changed: {}".format(
                    relative_path
                )
            )
        return total, digest.hexdigest(), b"".join(chunks)
    except EV3RuntimePreflightConfigurationError:
        raise
    except OSError:
        raise EV3RuntimePreflightConfigurationError(
            "Local manifest file is unreadable: {}".format(
                relative_path
            )
        ) from None
    finally:
        if handle is not None:
            handle.close()
        elif descriptor is not None:
            os.close(descriptor)


def _local_ancestor_tokens(
    local_root: Path,
    relative_path: str,
) -> Tuple[Tuple[int, int], ...]:
    current = local_root
    result = []
    for component in (None,) + Path(relative_path).parts[:-1]:
        if component is not None:
            current = current / component
        try:
            evidence = os.lstat(str(current))
        except OSError:
            raise EV3RuntimePreflightConfigurationError(
                "Local manifest ancestor is missing: {}".format(
                    relative_path
                )
            ) from None
        if (
            stat.S_ISLNK(evidence.st_mode)
            or not stat.S_ISDIR(evidence.st_mode)
        ):
            raise EV3RuntimePreflightConfigurationError(
                "Local manifest ancestor is unsafe: {}".format(
                    relative_path
                )
            )
        result.append((evidence.st_dev, evidence.st_ino))
    return tuple(result)


def _validate_local_content(
    relative_path: str,
    content: bytes,
) -> None:
    if relative_path.endswith(".py"):
        try:
            tree = ast.parse(
                content,
                filename=relative_path,
                feature_version=5,
            )
        except (SyntaxError, UnicodeError, ValueError):
            raise EV3RuntimePreflightConfigurationError(
                "Local runtime file is not Python 3.5 compatible: "
                "{}".format(relative_path)
            ) from None
        # CPython's best-effort feature grammar still accepts f-strings
        # when asked for 3.5, even though the EV3 interpreter cannot.
        if any(
            isinstance(node, (ast.JoinedStr, ast.FormattedValue))
            for node in ast.walk(tree)
        ):
            raise EV3RuntimePreflightConfigurationError(
                "Local runtime file is not Python 3.5 compatible: "
                "{}".format(relative_path)
            )
        for node in ast.walk(tree):
            if (
                isinstance(node, ast.ImportFrom)
                and node.module == "__future__"
                and any(
                    name.name == "annotations"
                    for name in node.names
                )
            ):
                raise EV3RuntimePreflightConfigurationError(
                    "Local runtime file is not Python 3.5 compatible: "
                    "{}".format(relative_path)
                )
        return
    if relative_path == "config/ev3rstorm.json":
        try:
            parsed = json.loads(
                content.decode("utf-8"),
                object_pairs_hook=_strict_object,
                parse_constant=_reject_constant,
            )
        except (UnicodeError, ValueError):
            raise EV3RuntimePreflightConfigurationError(
                "Local runtime config is not strict JSON: {}".format(
                    relative_path
                )
            ) from None
        if not isinstance(parsed, dict):
            raise EV3RuntimePreflightConfigurationError(
                "Local runtime config root is not an object: {}".format(
                    relative_path
                )
            )


def _local_manifest(
    local_root: object,
    manifest: Tuple[str, ...],
) -> Dict[str, Tuple[int, str]]:
    try:
        root = Path(local_root)
    except (TypeError, ValueError):
        raise EV3RuntimePreflightConfigurationError(
            "Local project root is invalid"
        ) from None
    if not root.is_dir():
        raise EV3RuntimePreflightConfigurationError(
            "Local project root is invalid"
        )
    _local_ancestor_tokens(root, manifest[0])
    result = {}
    for relative_path in manifest:
        size, digest, content = _hash_local_file(root, relative_path)
        _validate_local_content(relative_path, content)
        result[relative_path] = (size, digest)
    return result


def run_ev3_runtime_preflight(
    target: str,
    profile: str = "peripheral",
    local_root: object = ".",
    runner: Runner = subprocess.run,
    connect_timeout_seconds: int = 3,
    command_timeout_seconds: int = DEFAULT_COMMAND_TIMEOUT_SECONDS,
) -> Dict[str, object]:
    """Verify a fixed EV3 deployment profile without enabling motion."""

    validated_target = _validate_target(target)
    if (
        not isinstance(profile, str)
        or profile not in PROFILE_MANIFESTS
    ):
        raise EV3RuntimePreflightConfigurationError(
            "Deployment profile is invalid"
        )
    _validate_positive_int(
        "connect timeout",
        connect_timeout_seconds,
        30,
    )
    _validate_positive_int(
        "command timeout",
        command_timeout_seconds,
        MAX_COMMAND_TIMEOUT_SECONDS,
    )
    if not callable(runner):
        raise EV3RuntimePreflightConfigurationError(
            "SSH runner is invalid"
        )

    manifest = PROFILE_MANIFESTS[profile]
    local_files = _local_manifest(local_root, manifest)
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
        raise EV3RuntimePreflightTransportError(
            "Runtime preflight exceeded its command deadline"
        ) from None
    except UnicodeError:
        raise EV3RuntimePreflightProtocolError(
            "Runtime preflight returned invalid text"
        ) from None
    except OSError:
        raise EV3RuntimePreflightTransportError(
            "Could not start runtime preflight"
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
        raise EV3RuntimePreflightProtocolError(
            "SSH runner returned an invalid result"
        )
    if returncode != 0:
        raise EV3RuntimePreflightTransportError(
            "Runtime preflight failed with a nonzero status"
        )

    remote = _parse_remote_result(stdout)
    remote_by_path = {
        entry["path"]: entry
        for entry in remote["files"]
    }
    matched = []
    for relative_path in manifest:
        entry = remote_by_path[relative_path]
        status_value = entry["status"]
        if status_value != "ok":
            raise EV3RuntimeDeploymentMismatchError(
                "remote_{}".format(status_value),
                "Remote deployment file is {}: {}".format(
                    status_value.replace("_", " "),
                    relative_path,
                ),
            )
        local_size, local_digest = local_files[relative_path]
        if (
            entry["size"] != local_size
            or entry["sha256"] != local_digest
        ):
            raise EV3RuntimeDeploymentMismatchError(
                "hash_mismatch",
                "Remote deployment file is stale: {}".format(
                    relative_path
                ),
            )
        matched.append(
            {
                "path": relative_path,
                "size_bytes": local_size,
                "sha256": local_digest,
            }
        )

    return {
        "schema_version": 1,
        "status": "ready",
        "mode": "ev3-runtime-deployment-preflight",
        "effects": "read_only",
        "profile": profile,
        "file_count": len(matched),
        "files": matched,
    }
