"""Host-owned goal authority for user and idle navigation.

The motion supervisor owns individual motor pulses.  This module owns the
separate question of which high-level goal is allowed to create those pulses.
An idle model can therefore select from host-provided opportunities without
ever acquiring, transferring, or fabricating physical authority.
"""

from dataclasses import dataclass, field
import threading
import weakref
from typing import Optional

from .navigation_contract import (
    NavigationContractError,
    boolean,
    identifier,
    integer,
)
from .navigation_state import NavigationSnapshot


IDLE_EXPLORATION = "IDLE_EXPLORATION"
USER = "USER"


@dataclass(frozen=True)
class GoalLease:
    """One exclusive, cancelable grant to create a high-level goal."""

    lease_id: str
    owner: str
    generation: int
    goal_epoch: int
    plan_revision: int
    cancel_event: object = field(compare=False, repr=False)

    def __post_init__(self) -> None:
        identifier("lease_id", self.lease_id)
        if self.owner not in (IDLE_EXPLORATION, USER):
            raise NavigationContractError(
                "invalid_goal_owner",
                "Goal lease owner is invalid",
            )
        integer("generation", self.generation, 1, 2**63 - 1)
        integer("goal_epoch", self.goal_epoch, 1, 2**63 - 1)
        integer("plan_revision", self.plan_revision, 1, 2**63 - 1)
        if (
            not callable(getattr(self.cancel_event, "is_set", None))
            or not callable(getattr(self.cancel_event, "set", None))
        ):
            raise NavigationContractError(
                "invalid_lease_cancel_event",
                "Goal lease requires a cancel event",
            )


@dataclass(frozen=True)
class UserGoalReservation:
    """A user claim that blocks new idle work before planning finishes."""

    request_id: str
    generation: int

    def __post_init__(self) -> None:
        identifier("request_id", self.request_id)
        integer("generation", self.generation, 1, 2**63 - 1)


@dataclass(frozen=True)
class IdleDutyRearmGuard:
    """Exact in-process capability for one atomic duty-cycle re-arm."""

    generation: int

    def __post_init__(self) -> None:
        integer("generation", self.generation, 1, 2**63 - 1)


@dataclass(frozen=True)
class GoalAuthoritySnapshot:
    """Read-only diagnostics for dashboards and tests."""

    idle_enabled: bool
    faulted: bool
    active_owner: Optional[str]
    active_generation: Optional[int]
    pending_user_request_id: Optional[str]
    last_allocated_goal_epoch: int
    last_allocated_plan_revision: int

    def __post_init__(self) -> None:
        boolean("idle_enabled", self.idle_enabled)
        boolean("faulted", self.faulted)
        if self.active_owner is not None and self.active_owner not in (
            IDLE_EXPLORATION,
            USER,
        ):
            raise NavigationContractError(
                "invalid_active_owner",
                "Active goal owner is invalid",
            )
        if self.active_generation is not None:
            integer(
                "active_generation",
                self.active_generation,
                1,
                2**63 - 1,
            )
        if self.pending_user_request_id is not None:
            identifier(
                "pending_user_request_id",
                self.pending_user_request_id,
            )
        integer(
            "last_allocated_goal_epoch",
            self.last_allocated_goal_epoch,
            0,
            2**63 - 1,
        )
        integer(
            "last_allocated_plan_revision",
            self.last_allocated_plan_revision,
            0,
            2**63 - 1,
        )


class GoalLeaseCoordinator:
    """Serialize user and idle goal ownership for one robot controller.

    Reserving a user goal is intentionally separate from activating it.  The
    reservation atomically prevents new idle acquisition and cancels an
    existing idle lease.  Activation is permitted only after that lease has
    returned a verified, safe terminal stop.
    """

    def __init__(
        self,
        robot_id: str,
        controller_instance_id: str,
        starting_goal_epoch: int = 1,
        starting_plan_revision: int = 1,
        idle_enabled: bool = False,
    ):
        identifier("robot_id", robot_id)
        identifier("controller_instance_id", controller_instance_id)
        integer(
            "starting_goal_epoch",
            starting_goal_epoch,
            1,
            2**63 - 1,
        )
        integer(
            "starting_plan_revision",
            starting_plan_revision,
            1,
            2**63 - 1,
        )
        boolean("idle_enabled", idle_enabled)
        self.robot_id = robot_id
        self.controller_instance_id = controller_instance_id
        self._next_goal_epoch = starting_goal_epoch
        self._next_plan_revision = starting_plan_revision
        self._generation = 0
        self._idle_enabled = idle_enabled
        self._faulted = False
        self._active: Optional[GoalLease] = None
        self._pending_user: Optional[UserGoalReservation] = None
        self._idle_rearm_guard: Optional[IdleDutyRearmGuard] = None
        # Weak histories preserve exact-object idempotence for as long as a
        # caller can retry its reservation, without growing a strong-reference
        # replay cache throughout a long-running controller process.
        self._cancelled_users = weakref.WeakValueDictionary()
        self._activated_users = weakref.WeakValueDictionary()
        self._lock = threading.Lock()

    def _next_generation_locked(self) -> int:
        self._generation += 1
        return self._generation

    def _allocate_locked(self, owner: str) -> GoalLease:
        generation = self._next_generation_locked()
        lease = GoalLease(
            lease_id="{}-lease-{}".format(
                owner.lower().replace("_", "-"),
                generation,
            ),
            owner=owner,
            generation=generation,
            goal_epoch=self._next_goal_epoch,
            plan_revision=self._next_plan_revision,
            cancel_event=threading.Event(),
        )
        self._next_goal_epoch += 1
        self._next_plan_revision += 1
        self._active = lease
        return lease

    @property
    def state(self) -> GoalAuthoritySnapshot:
        with self._lock:
            active = self._active
            pending = self._pending_user
            return GoalAuthoritySnapshot(
                idle_enabled=self._idle_enabled,
                faulted=self._faulted,
                active_owner=None if active is None else active.owner,
                active_generation=(
                    None if active is None else active.generation
                ),
                pending_user_request_id=(
                    None if pending is None else pending.request_id
                ),
                last_allocated_goal_epoch=self._next_goal_epoch - 1,
                last_allocated_plan_revision=(
                    self._next_plan_revision - 1
                ),
            )

    def set_idle_enabled(self, enabled: bool) -> None:
        boolean("enabled", enabled)
        with self._lock:
            if enabled and self._idle_rearm_guard is not None:
                raise NavigationContractError(
                    "idle_duty_rearm_in_progress",
                    "Idle cannot be enabled during duty-cycle re-arm",
                )
            self._idle_enabled = enabled
            if (
                not enabled
                and self._active is not None
                and self._active.owner == IDLE_EXPLORATION
            ):
                self._active.cancel_event.set()

    def try_acquire_idle(self) -> Optional[GoalLease]:
        """Acquire idle authority, or return ``None`` without side effects."""

        with self._lock:
            if (
                not self._idle_enabled
                or self._faulted
                or self._active is not None
                or self._pending_user is not None
                or self._idle_rearm_guard is not None
            ):
                return None
            return self._allocate_locked(IDLE_EXPLORATION)

    def reserve_user(self, request_id: str) -> UserGoalReservation:
        """Block idle immediately and cancel any active idle lease."""

        identifier("request_id", request_id)
        with self._lock:
            if self._idle_rearm_guard is not None:
                raise NavigationContractError(
                    "idle_duty_rearm_in_progress",
                    "User reservation must wait for duty-cycle re-arm",
                )
            if self._faulted:
                raise NavigationContractError(
                    "goal_authority_faulted",
                    "Goal authority is faulted",
                )
            if self._pending_user is not None:
                if self._pending_user.request_id == request_id:
                    return self._pending_user
                raise NavigationContractError(
                    "user_goal_already_pending",
                    "Another user goal is already pending",
                )
            if (
                self._active is not None
                and self._active.owner == USER
            ):
                raise NavigationContractError(
                    "user_goal_already_active",
                    "A user goal is already active",
                )
            reservation = UserGoalReservation(
                request_id=request_id,
                generation=self._next_generation_locked(),
            )
            self._pending_user = reservation
            if self._active is not None:
                self._active.cancel_event.set()
            return reservation

    def cancel_user_reservation(
        self,
        reservation: UserGoalReservation,
    ) -> bool:
        """Cancel exactly one still-pending user claim.

        The reservation object is an in-process capability: a separately
        constructed value with the same fields is not accepted.  The first
        cancellation returns ``True`` and an immediate retry with that exact
        object returns ``False``.  A cancellation racing activation is
        serialized by the coordinator lock, so exactly one transition wins.

        Cancelling a claim that already interrupted idle work never clears the
        idle lease's cancellation event.  That lease must still return a
        verified terminal stop before idle can be acquired again.
        """

        if not isinstance(reservation, UserGoalReservation):
            raise NavigationContractError(
                "invalid_user_reservation",
                "User cancellation requires a typed reservation",
            )
        with self._lock:
            if self._pending_user is reservation:
                if (
                    self._pending_user.generation
                    != reservation.generation
                ):
                    raise NavigationContractError(
                        "stale_user_reservation",
                        "User reservation generation is no longer current",
                    )
                self._pending_user = None
                self._cancelled_users[
                    reservation.generation
                ] = reservation
                return True
            if (
                self._cancelled_users.get(reservation.generation)
                is reservation
            ):
                return False
            if (
                self._activated_users.get(reservation.generation)
                is reservation
            ):
                raise NavigationContractError(
                    "user_reservation_already_activated",
                    "Activated user reservation cannot be cancelled",
                )
            raise NavigationContractError(
                "stale_user_reservation",
                "User reservation is no longer current",
            )

    def activate_user(
        self,
        reservation: UserGoalReservation,
    ) -> GoalLease:
        """Activate a pending user goal after prior motion safely stopped."""

        if not isinstance(reservation, UserGoalReservation):
            raise NavigationContractError(
                "invalid_user_reservation",
                "User activation requires a typed reservation",
            )
        with self._lock:
            if self._idle_rearm_guard is not None:
                raise NavigationContractError(
                    "idle_duty_rearm_in_progress",
                    "User activation must wait for duty-cycle re-arm",
                )
            if self._faulted:
                raise NavigationContractError(
                    "goal_authority_faulted",
                    "Goal authority is faulted",
                )
            if self._pending_user is not reservation:
                raise NavigationContractError(
                    "stale_user_reservation",
                    "User reservation is no longer current",
                )
            if self._active is not None:
                raise NavigationContractError(
                    "goal_owner_still_active",
                    "Previous goal owner has not safely released",
                )
            self._pending_user = None
            self._activated_users[reservation.generation] = reservation
            return self._allocate_locked(USER)

    def begin_idle_duty_rearm(self) -> IdleDutyRearmGuard:
        """Atomically block every new goal while host budgets reset."""

        with self._lock:
            if (
                self._idle_enabled
                or self._faulted
                or self._active is not None
                or self._pending_user is not None
                or self._idle_rearm_guard is not None
            ):
                raise NavigationContractError(
                    "unsafe_idle_duty_rearm",
                    "Duty re-arm requires disabled idle and no goal claims",
                )
            guard = IdleDutyRearmGuard(
                generation=self._next_generation_locked(),
            )
            self._idle_rearm_guard = guard
            return guard

    def finish_idle_duty_rearm(
        self,
        guard: IdleDutyRearmGuard,
    ) -> None:
        """Release exactly the maintenance capability returned by begin."""

        if not isinstance(guard, IdleDutyRearmGuard):
            raise NavigationContractError(
                "invalid_idle_duty_rearm_guard",
                "Duty re-arm release requires a typed guard",
            )
        with self._lock:
            if self._idle_rearm_guard is not guard:
                raise NavigationContractError(
                    "stale_idle_duty_rearm_guard",
                    "Duty re-arm guard is no longer current",
                )
            self._idle_rearm_guard = None

    def is_current_idle(self, lease: GoalLease) -> bool:
        if not isinstance(lease, GoalLease):
            return False
        with self._lock:
            return (
                self._active is lease
                and lease.owner == IDLE_EXPLORATION
                and self._idle_enabled
                and not self._faulted
                and self._pending_user is None
                and not lease.cancel_event.is_set()
            )

    def release(
        self,
        lease: GoalLease,
        final_snapshot: NavigationSnapshot,
        terminal_stop_verified: bool,
    ) -> bool:
        """Release authority only after a verified safe stopped snapshot.

        An unverifiable release permanently faults this coordinator instance;
        it never silently re-arms autonomous motion.
        """

        if not isinstance(lease, GoalLease):
            raise NavigationContractError(
                "invalid_goal_lease",
                "Goal release requires a typed lease",
            )
        if not isinstance(final_snapshot, NavigationSnapshot):
            raise NavigationContractError(
                "invalid_final_snapshot",
                "Goal release requires NavigationSnapshot",
            )
        boolean("terminal_stop_verified", terminal_stop_verified)
        with self._lock:
            if self._active is not lease:
                raise NavigationContractError(
                    "stale_goal_lease",
                    "Goal lease is no longer active",
                )
            safe = (
                terminal_stop_verified
                and final_snapshot.robot_id == self.robot_id
                and final_snapshot.controller_instance_id
                == self.controller_instance_id
                and final_snapshot.goal_epoch == lease.goal_epoch
                and final_snapshot.plan_revision == lease.plan_revision
                and not final_snapshot.motors_running
                and not final_snapshot.touch_pressed
                and not final_snapshot.active_faults
            )
            self._active = None
            if not safe:
                self._faulted = True
                self._pending_user = None
            return safe


__all__ = (
    "GoalAuthoritySnapshot",
    "GoalLease",
    "GoalLeaseCoordinator",
    "IDLE_EXPLORATION",
    "IdleDutyRearmGuard",
    "USER",
    "UserGoalReservation",
)
