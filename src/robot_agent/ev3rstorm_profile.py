"""Explicit host composition for the assembled EV3RSTORM controller."""

from dataclasses import dataclass
import json
from pathlib import Path
import threading
from typing import Callable, Optional

from .active_ir_scan_contract import (
    ActiveIrScanCalibration,
    worst_case_scan_budget,
)
from .controller_runtime_profile import (
    ControllerRuntimeBinding,
    ControllerRuntimeDescriptor,
    ControllerRuntimeProfile,
    ControllerRuntimeProfileError,
)
from .ev3_active_ir_scan_rig import build_ev3_active_ir_scan_executor
from .ev3_audio_transport import EV3WAVSSHSession
from .ev3_navigation_transport import EV3NavigationSSHTransport
from .host_piper_speech import (
    HostPiperEV3Speaker,
    PiperLoopbackSynthesizer,
    PiperSpeechProfile,
)
from .navigation_memory_store import NavigationMemoryStore
from .physical_footprint import RobotFootprint
from .physical_navigation_adapter import PhysicalNavigationRuntimeAdapter
from .physical_odometry import OdometryCalibration
from .physical_spatial_map import PhysicalSpatialMapBridge
from .provisional_hazard_map import HazardMapCalibration
from .robot_speech_runtime import RobotSpeechRuntime


EV3RSTORM_PROFILE_ID = "ev3rstorm-01"
EV3RSTORM_REMOTE_WORKER_PATH = (
    "/home/robot/robot-llm/ev3/navigation_worker_cli.py"
)
EV3RSTORM_STARTUP_TIMEOUT_SECONDS = 30.0
EV3RSTORM_REQUEST_TIMEOUT_SECONDS = 30.0
EV3RSTORM_PLAN_TAIL_MAX_AGE_SECONDS = 45.0
# The EV3's coarse 90-degree action and asymmetric drive motors accumulated
# about 16 degrees of heading error in the first live detour.  This profile
# accepts that measured dead-reckoning precision while faster controllers can
# keep the generic five-degree default.
EV3RSTORM_GOAL_HEADING_TOLERANCE_MDEG = 20_000
# Live Wi-Fi evidence shows that EV3 worker operations vary from roughly
# 1.5 seconds for a stationary sample to more than 8 seconds for a sliced
# 90-degree turn. Spread measured controller/SSH headroom across the fixed
# request budget, then keep a separate cleanup reserve for stop, heading
# restoration and the final encoder observation.
EV3RSTORM_SCAN_REQUEST_ROUND_TRIP_HEADROOM_MS = 2_500
EV3RSTORM_SCAN_RESTORATION_HEADROOM_MS = 15_000
EV3RSTORM_SCAN_REQUEST_TIMEOUT_SECONDS = 30.0
EV3RSTORM_ACTIVE_IR_SCAN_CALIBRATION = ActiveIrScanCalibration(
    # Two live bilateral sweeps left the chassis about four and six degrees
    # from its starting heading.  EV3's coarse timed body turns therefore use
    # the contract's bounded ten-degree maximum.  Every ray still publishes
    # its exact encoder-derived bearing; other controllers retain the generic
    # 2.5-degree default.
    alignment_tolerance_mdeg=10_000,
)
EV3RSTORM_SCAN_BUDGET = worst_case_scan_budget(
    calibration=EV3RSTORM_ACTIVE_IR_SCAN_CALIBRATION,
    request_round_trip_headroom_ms=(
        EV3RSTORM_SCAN_REQUEST_ROUND_TRIP_HEADROOM_MS
    )
)
EV3RSTORM_SCAN_TIMEOUT_SECONDS = float(
    (
        EV3RSTORM_SCAN_BUDGET["minimum_deadline_ms"]
        + EV3RSTORM_SCAN_RESTORATION_HEADROOM_MS
        + 999
    )
    // 1_000
)
DEFAULT_EV3RSTORM_CONFIG_PATH = (
    Path(__file__).resolve().parents[2] / "config" / "ev3rstorm.json"
)
DEFAULT_EV3RSTORM_MEMORY_PATH = (
    Path.home()
    / ".robot-llm"
    / "navigation"
    / "ev3rstorm-01-memory.json"
)
MAX_PROFILE_CONFIG_BYTES = 128 * 1024


def _strict_object(pairs):
    value = {}
    for key, item in pairs:
        if key in value:
            raise ValueError("duplicate profile configuration key")
        value[key] = item
    return value


def _reject_constant(_value):
    raise ValueError("non-finite profile configuration value")


def _safe_target(value: object) -> str:
    if (
        not isinstance(value, str)
        or not value
        or value != value.strip()
        or len(value) > 255
        or value.startswith("-")
        or any(
            not (character.isalnum() or character in "._-@:%+")
            for character in value
        )
    ):
        raise ControllerRuntimeProfileError("EV3 SSH target is invalid")
    return value


def _safe_remote_path(value: object) -> str:
    allowed = (
        "abcdefghijklmnopqrstuvwxyz"
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "0123456789"
        "/._-"
    )
    if (
        not isinstance(value, str)
        or not value.startswith("/")
        or len(value) > 512
        or any(character not in allowed for character in value)
    ):
        raise ControllerRuntimeProfileError(
            "EV3 remote worker path is invalid"
        )
    return value


def _identity(value: object, name: str) -> str:
    if (
        not isinstance(value, str)
        or not value
        or value != value.strip()
        or len(value) > 128
        or any(ord(character) < 32 for character in value)
    ):
        raise ControllerRuntimeProfileError("{} is invalid".format(name))
    return value


def _physical_footprint(value: object) -> Optional[RobotFootprint]:
    # schema_version 1 predates host-side swept-volume geometry.  Existing
    # copied configs therefore retain the generic symmetric-circle fallback;
    # the checked-in assembled EV3RSTORM profile opts into the new shape.
    if value is None:
        return None
    expected = {
        "status",
        "reference_point",
        "front_extent_mm",
        "rear_extent_mm",
        "left_extent_mm",
        "right_extent_mm",
        "clearance_margin_mm",
        "evidence",
    }
    if not isinstance(value, dict) or set(value) != expected:
        raise ControllerRuntimeProfileError(
            "EV3RSTORM physical footprint is incomplete"
        )
    if (
        value["status"]
        != "provisional-unmeasured-operator-observed"
        or value["reference_point"] != "differential-drive-origin"
        or not isinstance(value["evidence"], str)
        or not value["evidence"]
        or len(value["evidence"]) > 512
    ):
        raise ControllerRuntimeProfileError(
            "EV3RSTORM physical footprint provenance is invalid"
        )
    try:
        return RobotFootprint(
            front_extent_mm=value["front_extent_mm"],
            rear_extent_mm=value["rear_extent_mm"],
            left_extent_mm=value["left_extent_mm"],
            right_extent_mm=value["right_extent_mm"],
            clearance_margin_mm=value["clearance_margin_mm"],
            calibration_status=value["status"],
            calibration_evidence=value["evidence"],
        )
    except ValueError as error:
        raise ControllerRuntimeProfileError(
            "EV3RSTORM physical footprint extents are invalid"
        ) from error


def _odometry_calibration(value: object) -> OdometryCalibration:
    expected = {
        "status",
        "linear_mm_per_encoder_degree",
        "turn_mdeg_per_opposed_encoder_degree",
        "evidence",
    }
    if (
        not isinstance(value, dict)
        or set(value) != expected
        or value["status"] != "provisional-live-encoder-derived"
        or not isinstance(value["evidence"], str)
        or not value["evidence"]
        or len(value["evidence"]) > 512
    ):
        raise ControllerRuntimeProfileError(
            "EV3RSTORM odometry calibration is incomplete"
        )
    try:
        return OdometryCalibration(
            linear_mm_per_encoder_degree=(
                value["linear_mm_per_encoder_degree"]
            ),
            turn_mdeg_per_opposed_encoder_degree=(
                value["turn_mdeg_per_opposed_encoder_degree"]
            ),
        )
    except (TypeError, ValueError) as error:
        raise ControllerRuntimeProfileError(
            "EV3RSTORM odometry calibration is invalid"
        ) from error


def _load_profile_config(path: Path):
    resolved = Path(path).expanduser().resolve()
    try:
        size = resolved.stat().st_size
        if not 1 <= size <= MAX_PROFILE_CONFIG_BYTES:
            raise ControllerRuntimeProfileError(
                "EV3RSTORM profile configuration size is invalid"
            )
        raw = resolved.read_text(encoding="utf-8")
        value = json.loads(
            raw,
            object_pairs_hook=_strict_object,
            parse_constant=_reject_constant,
        )
    except ControllerRuntimeProfileError:
        raise
    except (OSError, UnicodeError, ValueError) as error:
        raise ControllerRuntimeProfileError(
            "EV3RSTORM profile configuration could not be loaded: {}".format(
                error
            )
        ) from error
    if not isinstance(value, dict) or value.get("schema_version") != 1:
        raise ControllerRuntimeProfileError(
            "EV3RSTORM profile configuration schema is invalid"
        )
    robot_id = _identity(value.get("robot_id"), "robot_id")
    controller_id = _identity(
        value.get("controller_id"),
        "controller_id",
    )
    if robot_id != EV3RSTORM_PROFILE_ID:
        raise ControllerRuntimeProfileError(
            "EV3RSTORM configuration selected another robot"
        )
    motors = value.get("motors")
    sensors = value.get("sensors")
    geometry = value.get("drive_geometry")
    calibration = value.get("calibration")
    if (
        not isinstance(motors, dict)
        or not {"arm", "drive_b", "drive_c"} <= set(motors)
        or not isinstance(sensors, dict)
        or not {"touch", "infrared"} <= set(sensors)
        or not isinstance(geometry, dict)
        or geometry.get("left_motor_role") not in motors
        or geometry.get("right_motor_role") not in motors
        or not isinstance(calibration, dict)
    ):
        raise ControllerRuntimeProfileError(
            "EV3RSTORM topology is incomplete"
        )
    footprint = _physical_footprint(
        calibration.get("physical_footprint")
    )
    odometry = _odometry_calibration(calibration.get("odometry"))
    return resolved, robot_id, controller_id, footprint, odometry


@dataclass(frozen=True)
class EV3SSHBinding(ControllerRuntimeBinding):
    """Concrete location and host-local state for one EV3 deployment."""

    target: str
    memory_path: Path
    reset_memory: bool = False
    connect_timeout_seconds: int = 5
    remote_worker_path: str = EV3RSTORM_REMOTE_WORKER_PATH

    def __post_init__(self) -> None:
        super().__post_init__()
        object.__setattr__(self, "target", _safe_target(self.target))
        object.__setattr__(
            self,
            "remote_worker_path",
            _safe_remote_path(self.remote_worker_path),
        )
        resolved_memory = Path(self.memory_path).expanduser().resolve()
        if (
            resolved_memory.name in ("", ".", "..")
            or resolved_memory.exists()
            and not resolved_memory.is_file()
        ):
            raise ControllerRuntimeProfileError(
                "EV3 navigation memory path is invalid"
            )
        object.__setattr__(self, "memory_path", resolved_memory)
        if not isinstance(self.reset_memory, bool):
            raise ControllerRuntimeProfileError(
                "EV3 navigation memory reset flag is invalid"
            )
        if (
            isinstance(self.connect_timeout_seconds, bool)
            or not isinstance(self.connect_timeout_seconds, int)
            or not 1 <= self.connect_timeout_seconds <= 30
        ):
            raise ControllerRuntimeProfileError(
                "EV3 SSH connect timeout is invalid"
            )


class EV3RSTORMProfile(ControllerRuntimeProfile):
    """Build the existing EV3 runtime without touching the network."""

    def __init__(
        self,
        config_path: Path = DEFAULT_EV3RSTORM_CONFIG_PATH,
        speech_profile: PiperSpeechProfile = PiperSpeechProfile(),
    ):
        if not isinstance(speech_profile, PiperSpeechProfile):
            raise ControllerRuntimeProfileError(
                "EV3RSTORM speech profile is invalid"
            )
        (
            resolved,
            robot_id,
            controller_id,
            footprint,
            odometry,
        ) = _load_profile_config(config_path)
        self.config_path = resolved
        self.speech_profile = speech_profile
        self.hazard_calibration = HazardMapCalibration(
            robot_footprint=footprint,
        )
        self.odometry_calibration = odometry
        self._descriptor = ControllerRuntimeDescriptor(
            profile_id=EV3RSTORM_PROFILE_ID,
            robot_id=robot_id,
            controller_id=controller_id,
            display_name="EV3RSTORM",
            capabilities=(
                "motor.arm",
                "motor.drive.left",
                "motor.drive.right",
                "sensor.touch",
                "sensor.color",
                "sensor.infrared",
                "speaker.tts",
            ),
        )

    @property
    def descriptor(self) -> ControllerRuntimeDescriptor:
        return self._descriptor

    def build_adapter(
        self,
        binding,
        *,
        planner_factory: Callable[[str], object],
    ) -> PhysicalNavigationRuntimeAdapter:
        if not isinstance(binding, EV3SSHBinding):
            raise ControllerRuntimeProfileError(
                "EV3RSTORM requires an EV3 SSH binding"
            )
        if binding.profile_id != self.descriptor.profile_id:
            raise ControllerRuntimeProfileError(
                "EV3 binding selected another controller profile"
            )
        if not callable(planner_factory):
            raise ControllerRuntimeProfileError(
                "EV3 planner factory is invalid"
            )

        def transport_factory():
            return EV3NavigationSSHTransport(
                target=binding.target,
                controller_id=self.descriptor.controller_id,
                remote_worker_path=binding.remote_worker_path,
                connect_timeout_seconds=binding.connect_timeout_seconds,
            )

        reset_lock = threading.Lock()
        reset_pending = binding.reset_memory

        def memory_factory():
            nonlocal reset_pending
            # The CLI reset flag applies to the next successfully loaded
            # memory generation only. Persist it before consuming the flag so
            # a later worker cold-start failure cannot resurrect the old map
            # on the next episode. Serializing the load/save prevents
            # concurrent attempts from both erasing the same persisted map.
            with reset_lock:
                memory = NavigationMemoryStore.load(
                    path=binding.memory_path,
                    robot_id=self.descriptor.robot_id,
                    controller_instance_id=self.descriptor.controller_id,
                    reset=reset_pending,
                    odometry_calibration=self.odometry_calibration,
                    hazard_calibration=self.hazard_calibration,
                )
                if reset_pending:
                    memory.save()
                reset_pending = False
                return memory

        def speech_runtime_factory(*, event_sink):
            synthesizer = PiperLoopbackSynthesizer(self.speech_profile)
            speech_session = EV3WAVSSHSession(
                binding.target,
                connect_timeout_seconds=binding.connect_timeout_seconds,
            )
            speaker = HostPiperEV3Speaker(synthesizer, speech_session)
            return RobotSpeechRuntime(
                speaker=speaker,
                speaker_close=speech_session.close,
                event_sink=event_sink,
                thread_name="{}-speech".format(
                    self.descriptor.controller_id
                ),
            )

        def scan_executor_factory(transport):
            return build_ev3_active_ir_scan_executor(
                transport,
                request_timeout_seconds=(
                    EV3RSTORM_SCAN_REQUEST_TIMEOUT_SECONDS
                ),
                restoration_headroom_ms=(
                    EV3RSTORM_SCAN_RESTORATION_HEADROOM_MS
                ),
            )

        spatial_map_bridge = PhysicalSpatialMapBridge(
            robot_id=self.descriptor.robot_id,
            controller_instance_id=self.descriptor.controller_id,
        )

        return PhysicalNavigationRuntimeAdapter(
            transport_factory=transport_factory,
            planner_factory=planner_factory,
            memory_factory=memory_factory,
            scan_executor_factory=scan_executor_factory,
            speech_runtime_factory=speech_runtime_factory,
            spatial_map_bridge=spatial_map_bridge,
            goal_heading_tolerance_mdeg=(
                EV3RSTORM_GOAL_HEADING_TOLERANCE_MDEG
            ),
            startup_timeout_seconds=EV3RSTORM_STARTUP_TIMEOUT_SECONDS,
            request_timeout_seconds=EV3RSTORM_REQUEST_TIMEOUT_SECONDS,
            plan_tail_max_age_seconds=(
                EV3RSTORM_PLAN_TAIL_MAX_AGE_SECONDS
            ),
            scan_timeout_seconds=EV3RSTORM_SCAN_TIMEOUT_SECONDS,
            active_scan_calibration=(
                EV3RSTORM_ACTIVE_IR_SCAN_CALIBRATION
            ),
        )


__all__ = (
    "DEFAULT_EV3RSTORM_CONFIG_PATH",
    "DEFAULT_EV3RSTORM_MEMORY_PATH",
    "EV3RSTORM_PROFILE_ID",
    "EV3RSTORM_ACTIVE_IR_SCAN_CALIBRATION",
    "EV3RSTORM_GOAL_HEADING_TOLERANCE_MDEG",
    "EV3RSTORM_PLAN_TAIL_MAX_AGE_SECONDS",
    "EV3RSTORM_REMOTE_WORKER_PATH",
    "EV3RSTORM_REQUEST_TIMEOUT_SECONDS",
    "EV3RSTORM_SCAN_TIMEOUT_SECONDS",
    "EV3RSTORM_SCAN_RESTORATION_HEADROOM_MS",
    "EV3RSTORM_SCAN_REQUEST_TIMEOUT_SECONDS",
    "EV3RSTORM_STARTUP_TIMEOUT_SECONDS",
    "EV3RSTORMProfile",
    "EV3SSHBinding",
)
