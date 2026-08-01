"""Persistent local access key for the live Robot LLM console.

The key is an access capability, not a robot episode identity.  Keeping it in
an owner-only file lets the normal Mac launcher reuse one private console URL
across process restarts without making historical robot runs resumable.
"""

from __future__ import annotations

import os
from pathlib import Path
import secrets
import stat
from typing import Callable


ACCESS_KEY_BYTES = 32
MIN_ACCESS_KEY_CHARACTERS = 32
MAX_ACCESS_KEY_CHARACTERS = 128
MAX_ACCESS_KEY_FILE_BYTES = MAX_ACCESS_KEY_CHARACTERS + 1


class DashboardAccessKeyError(RuntimeError):
    pass


def _validate_access_key(value: object) -> str:
    if (
        not isinstance(value, str)
        or not MIN_ACCESS_KEY_CHARACTERS <= len(value)
        <= MAX_ACCESS_KEY_CHARACTERS
        or not value.isascii()
        or not all(character.isalnum() or character in "-_" for character in value)
    ):
        raise DashboardAccessKeyError("Dashboard access key is invalid")
    return value


def _open_flags(base: int) -> int:
    no_follow = getattr(os, "O_NOFOLLOW", 0)
    close_on_exec = getattr(os, "O_CLOEXEC", 0)
    return base | no_follow | close_on_exec


def _read_existing(path: Path) -> str:
    try:
        metadata = path.lstat()
    except OSError as error:
        raise DashboardAccessKeyError(
            "Dashboard access key could not be inspected"
        ) from error
    if (
        not stat.S_ISREG(metadata.st_mode)
        or metadata.st_uid != os.getuid()
        or metadata.st_mode & 0o077
        or not 1 <= metadata.st_size <= MAX_ACCESS_KEY_FILE_BYTES
    ):
        raise DashboardAccessKeyError(
            "Dashboard access key file must be owner-only and regular"
        )
    try:
        descriptor = os.open(path, _open_flags(os.O_RDONLY))
        try:
            opened = os.fstat(descriptor)
            if (
                not stat.S_ISREG(opened.st_mode)
                or opened.st_dev != metadata.st_dev
                or opened.st_ino != metadata.st_ino
            ):
                raise DashboardAccessKeyError(
                    "Dashboard access key file changed while opening"
                )
            raw = os.read(descriptor, MAX_ACCESS_KEY_FILE_BYTES + 1)
        finally:
            os.close(descriptor)
    except DashboardAccessKeyError:
        raise
    except OSError as error:
        raise DashboardAccessKeyError(
            "Dashboard access key could not be read"
        ) from error
    if len(raw) > MAX_ACCESS_KEY_FILE_BYTES:
        raise DashboardAccessKeyError("Dashboard access key file is too large")
    try:
        value = raw.decode("ascii")
    except UnicodeDecodeError as error:
        raise DashboardAccessKeyError(
            "Dashboard access key is not ASCII"
        ) from error
    if value.endswith("\n"):
        value = value[:-1]
    return _validate_access_key(value)


def load_or_create_dashboard_access_key(
    path: Path,
    *,
    token_factory: Callable[[], str] = lambda: secrets.token_hex(
        ACCESS_KEY_BYTES
    ),
) -> str:
    """Load one owner-only key or atomically create it on first launch."""

    supplied = Path(path).expanduser()
    if not supplied.name or supplied.name in (".", ".."):
        raise DashboardAccessKeyError("Dashboard access key path is invalid")
    absolute = Path(os.path.abspath(str(supplied)))
    try:
        absolute.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    except OSError as error:
        raise DashboardAccessKeyError(
            "Dashboard access key directory could not be created"
        ) from error

    try:
        absolute.lstat()
    except FileNotFoundError:
        pass
    except OSError as error:
        raise DashboardAccessKeyError(
            "Dashboard access key could not be inspected"
        ) from error
    else:
        return _read_existing(absolute)

    candidate = _validate_access_key(token_factory())
    try:
        descriptor = os.open(
            absolute,
            _open_flags(os.O_WRONLY | os.O_CREAT | os.O_EXCL),
            0o600,
        )
    except FileExistsError:
        return _read_existing(absolute)
    except OSError as error:
        raise DashboardAccessKeyError(
            "Dashboard access key could not be created"
        ) from error

    try:
        payload = (candidate + "\n").encode("ascii")
        written = 0
        while written < len(payload):
            count = os.write(descriptor, payload[written:])
            if count <= 0:
                raise OSError("short dashboard access key write")
            written += count
        os.fsync(descriptor)
    except OSError as error:
        try:
            os.close(descriptor)
        finally:
            try:
                absolute.unlink()
            except OSError:
                pass
        raise DashboardAccessKeyError(
            "Dashboard access key could not be stored"
        ) from error
    else:
        os.close(descriptor)
    return candidate


__all__ = (
    "DashboardAccessKeyError",
    "load_or_create_dashboard_access_key",
)
