"""One-shot fixed-start binding for two robot-local spatial maps.

The provider anchors its local robot at the origin of a new shared world and
places one peer with a configured SE(2) transform.  Binding happens only when
both sources expose a complete, compositor-valid local-map generation.  Once
bound, a source frame or generation change permanently requires an explicit
operator rebind (currently a fresh provider/process); it is never inferred.

Source providers are observation-only dependencies.  They are sampled once
per snapshot attempt and are neither written to nor closed here.
"""

from copy import deepcopy
import threading
from typing import Mapping, Optional, Tuple
from uuid import uuid4

from .shared_frame_transform import (
    CalibratedFrameTransform,
    FrameTransformError,
)
from .shared_spatial_map import (
    LATEST_AVAILABLE_NOT_ATOMIC,
    SHARED_FIXED_START,
    SHARED_SPATIAL_MAP_SCHEMA,
    SharedSpatialMapCompositor,
    SharedSpatialMapError,
)
from .spatial_map_contract import (
    DASHBOARD_SPATIAL_MAP_SCHEMA,
    LOCAL_ODOMETRY,
)


DEFAULT_WORLD_FRAME_ID = "shared-world"
DEFAULT_POSITION_UNCERTAINTY_MM = 25
DEFAULT_YAW_UNCERTAINTY_MDEG = 5_000
FIXED_START_SOURCES_PENDING = "fixed_start_sources_pending"
FIXED_START_REBIND_REQUIRED = "fixed_start_rebind_required"


class _SampledProvider:
    """Present one already captured value through the provider protocol."""

    def __init__(self, *, value=None, error: Optional[Exception] = None):
        self._value = value
        self._error = error

    def snapshot(self):
        if self._error is not None:
            raise self._error
        return self._value


def _sample(provider) -> _SampledProvider:
    try:
        return _SampledProvider(value=provider.snapshot())
    except Exception as error:
        return _SampledProvider(error=error)


def _identifier(value: object) -> Optional[str]:
    if (
        not isinstance(value, str)
        or not value
        or value != value.strip()
        or len(value) > 128
        or any(ord(character) < 32 for character in value)
    ):
        return None
    return value


def _source_frame_identity(
    sampled: _SampledProvider,
    *,
    robot_id: str,
    controller_id: str,
) -> Optional[Tuple[str, str]]:
    """Return a trustworthy local frame/generation identity, if present."""

    try:
        if sampled._error is not None:
            return None
        value = sampled._value
        if (
            not isinstance(value, Mapping)
            or value.get("schema") != DASHBOARD_SPATIAL_MAP_SCHEMA
            or value.get("read_only") is not True
            or value.get("robot_id") != robot_id
            or value.get("controller_instance_id") != controller_id
            or value.get("frame_kind") != LOCAL_ODOMETRY
        ):
            return None
        frame_id = _identifier(value.get("frame_id"))
        generation_id = _identifier(value.get("local_generation_id"))
    except Exception:
        return None
    if frame_id is None or generation_id is None:
        return None
    return frame_id, generation_id


class FixedStartSharedMapProvider:
    """Bind exactly one complete local/peer generation pair per instance."""

    def __init__(
        self,
        *,
        local_provider,
        peer_provider,
        local_robot_id: str,
        local_controller_id: str,
        peer_robot_id: str,
        peer_controller_id: str,
        peer_tx_mm: int,
        peer_ty_mm: int,
        peer_yaw_mdeg: int,
        world_frame_id: str = DEFAULT_WORLD_FRAME_ID,
        world_generation_id: Optional[str] = None,
        position_uncertainty_mm: int = DEFAULT_POSITION_UNCERTAINTY_MM,
        yaw_uncertainty_mdeg: int = DEFAULT_YAW_UNCERTAINTY_MDEG,
    ):
        try:
            local_snapshot = getattr(local_provider, "snapshot", None)
            peer_snapshot = getattr(peer_provider, "snapshot", None)
        except Exception:
            raise SharedSpatialMapError(
                "fixed-start map providers are invalid"
            ) from None
        if (
            not callable(local_snapshot)
            or not callable(peer_snapshot)
            or local_provider is peer_provider
            or (local_robot_id, local_controller_id)
            == (peer_robot_id, peer_controller_id)
        ):
            raise SharedSpatialMapError(
                "fixed-start map providers are invalid"
            )

        if world_generation_id is None:
            world_generation_id = "fixed-start-{}".format(uuid4().hex)

        # Validate the full static configuration at construction time.  The
        # source frame identities are deliberately placeholders until the
        # first complete generation pair is observed.
        try:
            local_template = CalibratedFrameTransform(
                source_robot_id=local_robot_id,
                source_controller_id=local_controller_id,
                source_frame_id="fixed-start-unbound-local-frame",
                source_generation_id="fixed-start-unbound-local-generation",
                world_frame_id=world_frame_id,
                world_generation_id=world_generation_id,
                tx_mm=0,
                ty_mm=0,
                yaw_mdeg=0,
                position_uncertainty_mm=position_uncertainty_mm,
                yaw_uncertainty_mdeg=yaw_uncertainty_mdeg,
                provenance=("FIXED_START_LOCAL_ANCHOR",),
            )
            peer_template = CalibratedFrameTransform(
                source_robot_id=peer_robot_id,
                source_controller_id=peer_controller_id,
                source_frame_id="fixed-start-unbound-peer-frame",
                source_generation_id="fixed-start-unbound-peer-generation",
                world_frame_id=world_frame_id,
                world_generation_id=world_generation_id,
                tx_mm=peer_tx_mm,
                ty_mm=peer_ty_mm,
                yaw_mdeg=peer_yaw_mdeg,
                position_uncertainty_mm=position_uncertainty_mm,
                yaw_uncertainty_mdeg=yaw_uncertainty_mdeg,
                provenance=("FIXED_START_PEER_SE2",),
            )
        except FrameTransformError:
            raise SharedSpatialMapError(
                "fixed-start map configuration is invalid"
            ) from None

        self._local_provider = local_provider
        self._peer_provider = peer_provider
        self._local_template = local_template
        self._peer_template = peer_template
        self._bound_transforms = None
        self._rebind_snapshot = None
        self._lock = threading.RLock()

    @property
    def world_frame_id(self) -> str:
        return self._local_template.world_frame_id

    @property
    def world_generation_id(self) -> str:
        return self._local_template.world_generation_id

    def _pending_snapshot(self):
        return {
            "schema": SHARED_SPATIAL_MAP_SCHEMA,
            "read_only": True,
            "status": "unavailable",
            "reason_code": FIXED_START_SOURCES_PENDING,
            "map_id": "{}.shared-fixed-start.{}".format(
                self.world_frame_id,
                self.world_generation_id,
            ),
            "frame_id": self.world_frame_id,
            "frame_kind": SHARED_FIXED_START,
            "world_generation_id": self.world_generation_id,
            "source_id": "fixed-start-shared-map-provider",
            "provenance": "CALIBRATED_FIXED_START_SE2_PROJECTION",
            "snapshot_semantics": LATEST_AVAILABLE_NOT_ATOMIC,
            "robots": [],
            "bounds": None,
            "cells": [],
            "sensor_rays": [],
            "qualitative_observations": [],
            "scan_evidence_history": [],
            "object_hypotheses": [],
            "navigation_authority": None,
            "captured_at_unix_ms": None,
        }

    @staticmethod
    def _bind_transform(
        template: CalibratedFrameTransform,
        identity: Tuple[str, str],
    ) -> CalibratedFrameTransform:
        frame_id, generation_id = identity
        return CalibratedFrameTransform(
            source_robot_id=template.source_robot_id,
            source_controller_id=template.source_controller_id,
            source_frame_id=frame_id,
            source_generation_id=generation_id,
            world_frame_id=template.world_frame_id,
            world_generation_id=template.world_generation_id,
            tx_mm=template.tx_mm,
            ty_mm=template.ty_mm,
            yaw_mdeg=template.yaw_mdeg,
            position_uncertainty_mm=template.position_uncertainty_mm,
            yaw_uncertainty_mdeg=template.yaw_uncertainty_mdeg,
            provenance=template.provenance,
        )

    def _compose(
        self,
        local_sample: _SampledProvider,
        peer_sample: _SampledProvider,
        transforms,
    ):
        local_transform, peer_transform = transforms
        return SharedSpatialMapCompositor(
            world_frame_id=self.world_frame_id,
            world_generation_id=self.world_generation_id,
            bindings=(
                (local_sample, local_transform),
                (peer_sample, peer_transform),
            ),
        ).snapshot()

    def _sample_sources(self):
        # Sample both even when the first fails.  A bind attempt is therefore
        # one bounded observation of each independently owned source.
        return (
            _sample(self._local_provider),
            _sample(self._peer_provider),
        )

    def snapshot(self):
        """Return a detached v2 snapshot without ever auto-rebinding."""

        with self._lock:
            if self._rebind_snapshot is not None:
                return deepcopy(self._rebind_snapshot)

            local_sample, peer_sample = self._sample_sources()
            local_identity = _source_frame_identity(
                local_sample,
                robot_id=self._local_template.source_robot_id,
                controller_id=self._local_template.source_controller_id,
            )
            peer_identity = _source_frame_identity(
                peer_sample,
                robot_id=self._peer_template.source_robot_id,
                controller_id=self._peer_template.source_controller_id,
            )

            if self._bound_transforms is None:
                if local_identity is None or peer_identity is None:
                    return deepcopy(self._pending_snapshot())
                candidate = (
                    self._bind_transform(self._local_template, local_identity),
                    self._bind_transform(self._peer_template, peer_identity),
                )
                value = self._compose(
                    local_sample,
                    peer_sample,
                    candidate,
                )
                if value.get("status") != "available":
                    return deepcopy(self._pending_snapshot())
                self._bound_transforms = candidate
                return value

            expected_local = (
                self._bound_transforms[0].source_frame_id,
                self._bound_transforms[0].source_generation_id,
            )
            expected_peer = (
                self._bound_transforms[1].source_frame_id,
                self._bound_transforms[1].source_generation_id,
            )
            generation_changed = (
                local_identity is not None
                and local_identity != expected_local
            ) or (
                peer_identity is not None
                and peer_identity != expected_peer
            )
            value = self._compose(
                local_sample,
                peer_sample,
                self._bound_transforms,
            )
            if generation_changed:
                value["status"] = "unavailable"
                value["reason_code"] = FIXED_START_REBIND_REQUIRED
                self._rebind_snapshot = deepcopy(value)
            return deepcopy(value)


__all__ = (
    "DEFAULT_POSITION_UNCERTAINTY_MM",
    "DEFAULT_WORLD_FRAME_ID",
    "DEFAULT_YAW_UNCERTAINTY_MDEG",
    "FIXED_START_REBIND_REQUIRED",
    "FIXED_START_SOURCES_PENDING",
    "FixedStartSharedMapProvider",
)
