"""Explicit host composition for the assembled EV3RSTORM controller."""

from dataclasses import asdict, dataclass
import json
from pathlib import Path
from types import SimpleNamespace
import threading
from typing import Optional

from .controller_runtime_profile import (
    ControllerRuntimeBinding,
    ControllerRuntimeDescriptor,
    ControllerRuntimeProfile,
    ControllerRuntimeProfileError,
)
from .ev3_audio_transport import EV3WAVSSHSession
from .ev3_social_speech import EV3SocialSpeaker
from .blast_personality import EV3_PERSONA_BY_LOCALE
from .social_expression import EV3_GESTURES
from .ev3_navigation_transport import EV3NavigationSSHTransport
from .ev3_navigation_body import EV3NavigationBody
from .ev3_navigation_preflight_cli import run_ev3_navigation_preflight
from .ev3_reachability import EV3ReachabilityProbe
from .host_piper_speech import (
    HostPiperEV3Speaker,
    PiperLoopbackSynthesizer,
    PiperSpeechProfile,
)
from .physical_footprint import RobotFootprint
from .physical_odometry import OdometryCalibration
from .physical_spatial_map import PhysicalSpatialMapBridge
from .ev3_episode_adapter import EV3EpisodeAdapter
from .lm_studio_controller_action import LMStudioControllerActionPlanner
from .provisional_hazard_map import HazardMapCalibration
from .robot_speech_runtime import RobotSpeechRuntime
from .robot_turn_speech import cancellable_serialized_speaker


EV3RSTORM_PROFILE_ID = "ev3rstorm-01"
EV3RSTORM_REMOTE_WORKER_PATH = (
    "/home/robot/robot-llm/ev3/navigation_worker_cli.py"
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
        not in {
            "provisional-unmeasured-operator-observed",
            "operator-measured-current-build",
        }
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


def _provisional_hazard_offset(value: object) -> int:
    # Existing schema-v1 copies predate this host-side map calibration.
    # Preserve their former HazardMapCalibration default until explicitly
    # measured, while the checked-in EV3 profile supplies live evidence.
    offset = value.get("provisional_hazard_offset_mm", 140) if isinstance(
        value,
        dict,
    ) else None
    if (
        isinstance(offset, bool)
        or not isinstance(offset, int)
        or not 50 <= offset <= 1_000
    ):
        raise ControllerRuntimeProfileError(
            "EV3RSTORM provisional hazard offset is invalid"
        )
    return offset


def _detour_lateral_clearance_margin(value: object) -> int:
    if value is None:
        return 0
    if (
        not isinstance(value, dict)
        or set(value) != {"lateral_clearance_margin_mm"}
    ):
        raise ControllerRuntimeProfileError(
            "EV3RSTORM local detour calibration is invalid"
        )
    margin = value["lateral_clearance_margin_mm"]
    if (
        isinstance(margin, bool)
        or not isinstance(margin, int)
        or not 0 <= margin <= 500
    ):
        raise ControllerRuntimeProfileError(
            "EV3RSTORM local detour calibration is invalid"
        )
    return margin


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
    hazard_offset = _provisional_hazard_offset(
        calibration.get("infrared_proximity")
    )
    detour_lateral_clearance_margin = (
        _detour_lateral_clearance_margin(
            calibration.get("local_detour")
        )
    )
    return (
        resolved,
        robot_id,
        controller_id,
        footprint,
        odometry,
        hazard_offset,
        detour_lateral_clearance_margin,
    )


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
    """Compose EV3 hardware and speech without an independent route planner."""

    def __init__(
        self,
        config_path: Path = DEFAULT_EV3RSTORM_CONFIG_PATH,
        speech_profile: PiperSpeechProfile = PiperSpeechProfile(
            voices=(("sv", "nst-deep"), ("en", "john-medium")),
        ),
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
            hazard_offset,
            detour_lateral_clearance_margin,
        ) = _load_profile_config(config_path)
        self.config_path = resolved
        self.speech_profile = speech_profile
        self._speech_lock = threading.Lock()
        self.hazard_calibration = HazardMapCalibration(
            provisional_hazard_offset_mm=hazard_offset,
            detour_lateral_clearance_margin_mm=(
                detour_lateral_clearance_margin
            ),
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

    def build_adapter(self, binding, *, planner_factory=None):
        """Compose EV3 with the shared planner; no connection or motion at construction."""
        if not isinstance(binding, EV3SSHBinding):
            raise ControllerRuntimeProfileError("EV3RSTORM requires an EV3 SSH binding")
        if binding.profile_id != self.descriptor.profile_id:
            raise ControllerRuntimeProfileError("EV3 binding selected another controller profile")
        if planner_factory is None:
            planner_factory = lambda model: LMStudioControllerActionPlanner(
                model=model, max_output_tokens=4096, timeout_seconds=60,
                social_expressions=True, social_gestures=EV3_GESTURES, social_faces=("neutral",),
                utterance_persona_by_locale=EV3_PERSONA_BY_LOCALE)

        def transport_factory():
            return EV3NavigationSSHTransport(
                target=binding.target,
                controller_id=self.descriptor.controller_id,
                remote_worker_path=binding.remote_worker_path,
                connect_timeout_seconds=binding.connect_timeout_seconds,
            )

        def speech_runtime_factory(*, event_sink, gesture_allowed=None):
            synthesizer = PiperLoopbackSynthesizer(self.speech_profile)
            speech_session = EV3WAVSSHSession(
                binding.target,
                connect_timeout_seconds=binding.connect_timeout_seconds,
            )
            speaker = cancellable_serialized_speaker(
                EV3SocialSpeaker(
                    HostPiperEV3Speaker(synthesizer, speech_session),
                    binding.target, controller_lease, gesture_allowed=gesture_allowed,
                ), self._speech_lock,
            )
            return RobotSpeechRuntime(
                speaker=speaker, speaker_close=speech_session.close,
                event_sink=event_sink,
                thread_name="{}-speech".format(self.descriptor.controller_id),
            )

        controller_lease = threading.Lock()
        reachability = EV3ReachabilityProbe(
            robot_id=self.descriptor.robot_id,
            controller_id=self.descriptor.controller_id,
            display_name=self.descriptor.display_name,
            controller_lease=controller_lease,
            preflight=lambda: run_ev3_navigation_preflight(
                binding.target,
                controller_id=self.descriptor.controller_id,
                remote_worker_path=binding.remote_worker_path,
                connect_timeout_seconds=binding.connect_timeout_seconds,
            ),
        )
        description = asdict(self.descriptor)
        body_factory = lambda **options: EV3NavigationBody(transport_factory, calibration=self.odometry_calibration, **options)
        map_provider = PhysicalSpatialMapBridge(robot_id=self.descriptor.robot_id, controller_instance_id=self.descriptor.controller_id)
        navigation = EV3EpisodeAdapter(body_factory=body_factory, planner_factory=planner_factory,
            descriptor=self.descriptor, footprint=self.hazard_calibration.robot_footprint,
            spatial_map_provider=map_provider, speech_runtime_factory=speech_runtime_factory, controller_lease=controller_lease)
        return SimpleNamespace(
            navigation_adapter=navigation,
            transport_factory=transport_factory,
            navigation_body_factory=body_factory,
            speech_runtime_factory=speech_runtime_factory,
            speech_locales=("sv", "en"),
            spatial_map_provider=map_provider,
            controller_lease=controller_lease,
            controller_runtime_provider=reachability,
            controller_reachability_service=reachability,
            robot_description=description,
        )


__all__ = (
    "DEFAULT_EV3RSTORM_CONFIG_PATH",
    "DEFAULT_EV3RSTORM_MEMORY_PATH",
    "EV3RSTORM_PROFILE_ID",
    "EV3RSTORM_REMOTE_WORKER_PATH",
    "EV3RSTORMProfile",
    "EV3SSHBinding",
)
