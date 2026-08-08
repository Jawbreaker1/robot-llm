"""Truthful, motion-free reachability state for an episodic EV3 SSH link."""

from copy import deepcopy
import threading
import time
from typing import Callable, Mapping

from .ev3_navigation_preflight_cli import EV3NavigationPreflightError


SNAPSHOT_SCHEMA = "controller-runtime-observation/v1"


class EV3ReachabilityError(RuntimeError):
    """Sanitized reachability failure suitable for the HTTP boundary."""

    def __init__(self, code: str, message: str):
        self.code = code
        super().__init__(message)


def _identity(name: str, value: object) -> str:
    if (
        not isinstance(value, str)
        or not value
        or value != value.strip()
        or len(value) > 128
        or any(ord(character) < 32 for character in value)
    ):
        raise ValueError("{} is invalid".format(name))
    return value


class EV3ReachabilityProbe:
    """Run one bounded preflight without claiming a persistent connection."""

    def __init__(
        self,
        *,
        robot_id: str,
        controller_id: str,
        display_name: str,
        controller_lease,
        preflight: Callable[[], Mapping[str, object]],
        clock_ms: Callable[[], int] = lambda: int(time.time() * 1_000),
    ):
        if any(
            not callable(getattr(controller_lease, name, None))
            for name in ("acquire", "release")
        ) or not callable(preflight) or not callable(clock_ms):
            raise ValueError("EV3 reachability configuration is invalid")
        self._lease = controller_lease
        self._preflight = preflight
        self._clock_ms = clock_ms
        self._lock = threading.Lock()
        self._snapshot = {
            "schema": SNAPSHOT_SCHEMA,
            "robot_id": _identity("robot_id", robot_id),
            "controller_id": _identity("controller_id", controller_id),
            "display_name": _identity("display_name", display_name),
            "state": "configured",
            "reason_code": "reachability_not_checked",
            "connection_mode": "episodic_ssh",
            "reachability": {
                "status": "not_checked",
                "error_code": None,
            },
            "last_checked_at_unix_ms": None,
            "last_verified_at_unix_ms": None,
            "observation": None,
        }

    def snapshot(self):
        with self._lock:
            return deepcopy(self._snapshot)

    def _set_reachability(
        self,
        status: str,
        reason_code: str,
        *,
        error_code=None,
        **changes
    ) -> None:
        with self._lock:
            self._snapshot.update(
                reason_code=reason_code,
                reachability={
                    "status": status,
                    "error_code": error_code,
                },
                **changes,
            )

    def check(self):
        if not self._lease.acquire(blocking=False):
            raise EV3ReachabilityError(
                "controller_busy",
                "EV3 is already in use",
            )
        try:
            self._set_reachability(
                "checking",
                "reachability_check_running",
            )
            try:
                report = self._preflight()
                checks = (
                    report.get("contract_checks")
                    if isinstance(report, Mapping)
                    else None
                )
                if (
                    not isinstance(report, Mapping)
                    or report.get("status") != "passed"
                    or report.get("effects") != "motion_free"
                    or not isinstance(checks, Mapping)
                    or checks.get("motor_commands_issued") != 0
                    or checks.get("shutdown_confirmed") is not True
                    or checks.get("motor_owner_closed") is not True
                ):
                    raise ValueError("invalid preflight report")
            except Exception as error:
                checked_at = self._clock_ms()
                reason = (
                    error.code
                    if isinstance(error, EV3NavigationPreflightError)
                    else "reachability_check_failed"
                )
                self._set_reachability(
                    "failed",
                    reason,
                    error_code=reason,
                    last_checked_at_unix_ms=checked_at,
                )
                raise EV3ReachabilityError(
                    "controller_connection_failed",
                    "EV3 reachability check failed",
                ) from None

            checked_at = self._clock_ms()
            self._set_reachability(
                "passed",
                "reachability_verified",
                last_checked_at_unix_ms=checked_at,
                last_verified_at_unix_ms=checked_at,
            )
            return self.snapshot()
        finally:
            self._lease.release()


__all__ = (
    "EV3ReachabilityError",
    "EV3ReachabilityProbe",
    "SNAPSHOT_SCHEMA",
)
