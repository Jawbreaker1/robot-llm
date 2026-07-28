#!/usr/bin/env python3
"""Strict Python 3.5-compatible loader for the EV3 operational config."""

from __future__ import print_function

import io
import json
import math


SCHEMA_VERSION = 1
TOP_LEVEL_KEYS = frozenset(
    (
        "schema_version",
        "robot_id",
        "controller_id",
        "motors",
        "drive_geometry",
        "agent_api",
        "calibration",
        "sensors",
        "limits",
    )
)
MOTION_LIMIT_PROFILES = frozenset(("drive", "arm"))
MOTOR_PORTS = frozenset(("outA", "outB", "outC", "outD"))
SENSOR_PORTS = frozenset(("in1", "in2", "in3", "in4"))
MAX_MOTOR_SPEED_DPS = 1560
MAX_MOTION_DURATION_MS = 5000
MAX_SPEECH_CHARACTERS = 1000
MAX_SPEECH_RATE_WPM = 500
MAX_SPEECH_AMPLITUDE = 200
MAX_SPEECH_RUNTIME_MS = 60000
MAX_HEARTBEAT_TIMEOUT_MS = 2000
MIN_STOP_STABLE_WINDOW_MS = 50
MIN_STOP_STABLE_INTERVALS = 3
MAX_IR_MEDIAN_WINDOW = 31
MAX_IR_CONSECUTIVE_DECISIONS = 20
MOTOR_DRIVER_MAX_SPEED_DPS = {
    "lego-ev3-l-motor": 1050,
    "lego-ev3-m-motor": 1560,
}
SUPERVISOR_LIMIT_KEYS = frozenset(
    (
        "poll_interval_ms",
        "max_poll_lateness_ms",
        "touch_release_samples",
        "min_abs_drive_speed_dps",
        "stall_startup_grace_ms",
        "stall_window_ms",
        "stall_min_progress_degrees",
        "stall_min_progress_ratio_percent",
        "min_completion_ratio_percent",
        "max_start_skew_ms",
        "stop_verify_timeout_ms",
        "stop_poll_interval_ms",
        "max_commands_per_session",
        "audit_buffer_events",
    )
)
SUPERVISOR_LIMIT_MAXIMUMS = {
    "poll_interval_ms": 100,
    "max_poll_lateness_ms": 500,
    "touch_release_samples": 100,
    "min_abs_drive_speed_dps": MAX_MOTOR_SPEED_DPS,
    "stall_startup_grace_ms": MAX_MOTION_DURATION_MS,
    "stall_window_ms": MAX_MOTION_DURATION_MS,
    "stall_min_progress_degrees": 1000,
    "stall_min_progress_ratio_percent": 100,
    "min_completion_ratio_percent": 100,
    "max_start_skew_ms": 100,
    "stop_verify_timeout_ms": 2000,
    "stop_poll_interval_ms": 100,
    "max_commands_per_session": 10000,
    "audit_buffer_events": 10000,
}


def _strict_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(
                "Duplicate configuration key {!r}".format(key)
            )
        result[key] = value
    return result


def _reject_constant(value):
    raise ValueError(
        "Non-finite configuration value {!r}".format(value)
    )


def _is_int(value):
    return isinstance(value, int) and not isinstance(value, bool)


def _require_object(value, path):
    if not isinstance(value, dict):
        raise ValueError("{} must be an object".format(path))
    return value


def _require_exact_keys(value, path, required, optional=()):
    value = _require_object(value, path)
    required = frozenset(required)
    allowed = required | frozenset(optional)
    actual = frozenset(value)
    missing = sorted(required - actual)
    unknown = sorted(actual - allowed)
    if missing:
        raise ValueError(
            "{} is missing required key(s): {}".format(
                path, ", ".join(missing)
            )
        )
    if unknown:
        raise ValueError(
            "{} contains unknown key(s): {}".format(
                path, ", ".join(unknown)
            )
        )
    return value


def _safe_string(value, path, maximum=128):
    if (
        not isinstance(value, str)
        or not value
        or value != value.strip()
        or len(value) > maximum
        or "\x00" in value
        or "\n" in value
        or "\r" in value
    ):
        raise ValueError(
            "{} must contain 1..{} safe characters".format(path, maximum)
        )
    return value


def _integer(value, path, minimum=1, maximum=None):
    if not _is_int(value) or value < minimum:
        if minimum == 1:
            detail = "a positive integer"
        else:
            detail = "an integer greater than or equal to {}".format(
                minimum
            )
        raise ValueError("{} must be {}".format(path, detail))
    if maximum is not None and value > maximum:
        raise ValueError(
            "{} must be at most {}".format(path, maximum)
        )
    return value


def _validate_finite_tree(value, path):
    if isinstance(value, float) and not math.isfinite(value):
        raise ValueError("{} must be finite".format(path))
    if isinstance(value, dict):
        for key, child in value.items():
            _validate_finite_tree(
                child, "{}.{}".format(path, key)
            )
    elif isinstance(value, list):
        for index, child in enumerate(value):
            _validate_finite_tree(
                child, "{}[{}]".format(path, index)
            )


def _validate_motors(config):
    motors = _require_object(config["motors"], "motors")
    if not motors:
        raise ValueError("motors must contain at least one role")

    ports = {}
    for role, details in motors.items():
        role_path = "motors.{}".format(role)
        _safe_string(role, "{} role".format(role_path))
        details = _require_exact_keys(
            details,
            role_path,
            ("port", "driver", "limit_profile"),
            ("physical_role", "positive_direction"),
        )
        port = details["port"]
        if not isinstance(port, str) or port not in MOTOR_PORTS:
            raise ValueError(
                "{}.port must be one of {}".format(
                    role_path, sorted(MOTOR_PORTS)
                )
            )
        if port in ports:
            raise ValueError(
                "Motor roles {!r} and {!r} share port {}".format(
                    ports[port], role, port
                )
            )
        ports[port] = role
        driver = _safe_string(
            details["driver"], "{}.driver".format(role_path)
        )
        if driver not in MOTOR_DRIVER_MAX_SPEED_DPS:
            raise ValueError(
                "{}.driver is not a supported EV3 tacho driver".format(
                    role_path
                )
            )
        if details["limit_profile"] not in MOTION_LIMIT_PROFILES:
            raise ValueError(
                "{}.limit_profile must be drive or arm".format(role_path)
            )
        configured_speed = config["limits"][
            details["limit_profile"]
        ]["max_abs_speed_dps"]
        if configured_speed > MOTOR_DRIVER_MAX_SPEED_DPS[driver]:
            raise ValueError(
                "{} uses a speed limit above driver maximum {}".format(
                    role_path, MOTOR_DRIVER_MAX_SPEED_DPS[driver]
                )
            )
        for metadata_name in ("physical_role", "positive_direction"):
            if metadata_name in details:
                _safe_string(
                    details[metadata_name],
                    "{}.{}".format(role_path, metadata_name),
                    256,
                )
    return motors


def _validate_sensors(config):
    sensors = _require_object(config["sensors"], "sensors")
    if not sensors:
        raise ValueError("sensors must contain at least one role")

    ports = {}
    for role, details in sensors.items():
        role_path = "sensors.{}".format(role)
        _safe_string(role, "{} role".format(role_path))
        details = _require_exact_keys(
            details,
            role_path,
            ("port", "driver"),
            ("mode",),
        )
        port = details["port"]
        if not isinstance(port, str) or port not in SENSOR_PORTS:
            raise ValueError(
                "{}.port must be one of {}".format(
                    role_path, sorted(SENSOR_PORTS)
                )
            )
        if port in ports:
            raise ValueError(
                "Sensor roles {!r} and {!r} share port {}".format(
                    ports[port], role, port
                )
            )
        ports[port] = role
        _safe_string(details["driver"], "{}.driver".format(role_path))
        if "mode" in details:
            _safe_string(details["mode"], "{}.mode".format(role_path))

    touch = sensors.get("touch")
    if touch is None:
        raise ValueError(
            "sensors.touch is required by the local safety supervisor"
        )
    if touch["driver"] != "lego-ev3-touch":
        raise ValueError(
            "sensors.touch.driver must be lego-ev3-touch"
        )
    if touch.get("mode") != "TOUCH":
        raise ValueError("sensors.touch.mode must be TOUCH")
    return sensors


def _validate_motion_limit(limits, profile):
    path = "limits.{}".format(profile)
    values = _require_exact_keys(
        limits[profile],
        path,
        ("max_abs_speed_dps", "max_duration_ms"),
    )
    speed = _integer(
        values["max_abs_speed_dps"],
        "{}.max_abs_speed_dps".format(path),
        maximum=MAX_MOTOR_SPEED_DPS,
    )
    duration = _integer(
        values["max_duration_ms"],
        "{}.max_duration_ms".format(path),
        maximum=MAX_MOTION_DURATION_MS,
    )
    if speed * duration < 3000:
        raise ValueError(
            "{} cannot admit an encoder-verifiable motion".format(path)
        )


def _validate_speech_limits(limits):
    path = "limits.speech"
    speech = _require_exact_keys(
        limits["speech"],
        path,
        (
            "max_characters",
            "min_rate_wpm",
            "max_rate_wpm",
            "min_amplitude",
            "max_amplitude",
            "default_amplitude",
            "allowed_voices",
            "max_runtime_ms",
        ),
    )
    maximums = {
        "max_characters": MAX_SPEECH_CHARACTERS,
        "min_rate_wpm": MAX_SPEECH_RATE_WPM,
        "max_rate_wpm": MAX_SPEECH_RATE_WPM,
        "min_amplitude": MAX_SPEECH_AMPLITUDE,
        "max_amplitude": MAX_SPEECH_AMPLITUDE,
        "default_amplitude": MAX_SPEECH_AMPLITUDE,
        "max_runtime_ms": MAX_SPEECH_RUNTIME_MS,
    }
    for name, maximum in maximums.items():
        minimum = 0 if name in (
            "min_amplitude",
            "default_amplitude",
        ) else 1
        _integer(
            speech[name],
            "{}.{}".format(path, name),
            minimum=minimum,
            maximum=maximum,
        )
    if speech["min_rate_wpm"] > speech["max_rate_wpm"]:
        raise ValueError(
            "{} rate minimum exceeds its maximum".format(path)
        )
    if speech["min_amplitude"] > speech["max_amplitude"]:
        raise ValueError(
            "{} amplitude minimum exceeds its maximum".format(path)
        )
    if not (
        speech["min_amplitude"]
        <= speech["default_amplitude"]
        <= speech["max_amplitude"]
    ):
        raise ValueError(
            "{}.default_amplitude is outside the configured range".format(
                path
            )
        )

    voices = speech["allowed_voices"]
    if (
        not isinstance(voices, list)
        or not voices
        or len(voices) > 32
    ):
        raise ValueError(
            "{}.allowed_voices must contain 1..32 voices".format(path)
        )
    seen = set()
    for index, voice in enumerate(voices):
        _safe_string(
            voice,
            "{}.allowed_voices[{}]".format(path, index),
            64,
        )
        if voice in seen:
            raise ValueError(
                "{}.allowed_voices must be unique".format(path)
            )
        seen.add(voice)


def _validate_supervisor_limits(limits):
    supervisor = _require_exact_keys(
        limits["supervisor"],
        "limits.supervisor",
        SUPERVISOR_LIMIT_KEYS,
    )
    for name in SUPERVISOR_LIMIT_KEYS:
        _integer(
            supervisor[name],
            "limits.supervisor.{}".format(name),
            maximum=SUPERVISOR_LIMIT_MAXIMUMS[name],
        )
    if supervisor["audit_buffer_events"] < 2:
        raise ValueError(
            "limits.supervisor.audit_buffer_events must be at least 2"
        )
    if (
        supervisor["stop_poll_interval_ms"]
        > supervisor["stop_verify_timeout_ms"]
    ):
        raise ValueError(
            "Supervisor stop poll interval exceeds stop timeout"
        )
    required_stable_intervals = max(
        MIN_STOP_STABLE_INTERVALS,
        (
            MIN_STOP_STABLE_WINDOW_MS
            + supervisor["stop_poll_interval_ms"]
            - 1
        )
        // supervisor["stop_poll_interval_ms"],
    )
    if (
        required_stable_intervals
        * supervisor["stop_poll_interval_ms"]
        > supervisor["stop_verify_timeout_ms"]
    ):
        raise ValueError(
            "Supervisor stop timeout cannot contain the required "
            "settling window"
        )

    heartbeat_timeout = limits["heartbeat"]["timeout_ms"]
    if supervisor["poll_interval_ms"] >= heartbeat_timeout:
        raise ValueError(
            "Supervisor poll interval must be shorter than heartbeat timeout"
        )
    if supervisor["max_poll_lateness_ms"] >= heartbeat_timeout:
        raise ValueError(
            "Supervisor poll lateness must be shorter than heartbeat timeout"
        )
    if (
        supervisor["min_abs_drive_speed_dps"]
        > limits["drive"]["max_abs_speed_dps"]
    ):
        raise ValueError(
            "Supervisor drive speed floor exceeds the drive speed limit"
        )
    for name in (
        "stall_min_progress_ratio_percent",
        "min_completion_ratio_percent",
    ):
        if supervisor[name] > 100:
            raise ValueError(
                "limits.supervisor.{} must be at most 100".format(name)
            )
    if (
        supervisor["stall_startup_grace_ms"]
        + supervisor["stall_window_ms"]
        > limits["drive"]["max_duration_ms"]
    ):
        raise ValueError(
            "Supervisor stall detection exceeds maximum drive duration"
        )


def _validate_limits(config):
    limits = _require_exact_keys(
        config["limits"],
        "limits",
        ("drive", "arm", "speech", "heartbeat", "supervisor"),
    )
    _validate_motion_limit(limits, "drive")
    _validate_motion_limit(limits, "arm")
    _validate_speech_limits(limits)
    heartbeat = _require_exact_keys(
        limits["heartbeat"],
        "limits.heartbeat",
        ("timeout_ms",),
    )
    _integer(
        heartbeat["timeout_ms"],
        "limits.heartbeat.timeout_ms",
        maximum=MAX_HEARTBEAT_TIMEOUT_MS,
    )
    _validate_supervisor_limits(limits)
    return limits


def _validate_drive_geometry(config, motors):
    geometry = _require_exact_keys(
        config["drive_geometry"],
        "drive_geometry",
        (
            "left_motor_role",
            "right_motor_role",
            "forward_speed_sign",
        ),
    )
    left_role = _safe_string(
        geometry["left_motor_role"],
        "drive_geometry.left_motor_role",
    )
    right_role = _safe_string(
        geometry["right_motor_role"],
        "drive_geometry.right_motor_role",
    )
    if left_role == right_role:
        raise ValueError("Drive motor roles must be different")
    for role in (left_role, right_role):
        if role not in motors:
            raise ValueError(
                "Drive geometry references unknown motor role {!r}".format(
                    role
                )
            )
        if motors[role]["limit_profile"] != "drive":
            raise ValueError(
                "Drive role {!r} must use the drive limit profile".format(
                    role
                )
            )

    signs = _require_exact_keys(
        geometry["forward_speed_sign"],
        "drive_geometry.forward_speed_sign",
        (left_role, right_role),
    )
    for role in (left_role, right_role):
        sign = signs[role]
        if not _is_int(sign) or sign not in (-1, 1):
            raise ValueError(
                "Forward speed sign for {!r} must be -1 or 1".format(role)
            )
    return frozenset((left_role, right_role))


def _validate_agent_api(config, motors, drive_roles):
    agent_api = _require_exact_keys(
        config["agent_api"],
        "agent_api",
        ("move_motor_roles",),
    )
    roles = agent_api["move_motor_roles"]
    if not isinstance(roles, list) or not roles:
        raise ValueError(
            "agent_api.move_motor_roles must be a non-empty list"
        )
    seen = set()
    for index, role in enumerate(roles):
        _safe_string(
            role,
            "agent_api.move_motor_roles[{}]".format(index),
        )
        if role not in motors:
            raise ValueError(
                "agent_api references unknown motor role {!r}".format(role)
            )
        if role in drive_roles:
            raise ValueError(
                "Drive role {!r} may not be exposed as an auxiliary motor".format(
                    role
                )
            )
        if motors[role]["limit_profile"] == "drive":
            raise ValueError(
                "Auxiliary role {!r} may not use drive limits".format(role)
            )
        if role in seen:
            raise ValueError(
                "agent_api.move_motor_roles must contain unique roles"
            )
        seen.add(role)


def _validate_ir_calibration(config, sensors):
    calibration = _require_object(config["calibration"], "calibration")
    if "infrared" not in sensors:
        return
    try:
        infrared = calibration["infrared_proximity"]
        gate = infrared["obstacle_gate"]
        zones = infrared["zones"]
    except (KeyError, TypeError):
        raise ValueError(
            "calibration.infrared_proximity is incomplete"
        )
    infrared = _require_object(
        infrared, "calibration.infrared_proximity"
    )
    mode = _safe_string(
        infrared.get("mode"),
        "calibration.infrared_proximity.mode",
    )
    sensor_mode = sensors["infrared"].get("mode")
    if sensor_mode is not None and sensor_mode != mode:
        raise ValueError(
            "Infrared calibration mode does not match the sensor mode"
        )

    gate = _require_exact_keys(
        gate,
        "calibration.infrared_proximity.obstacle_gate",
        (
            "immediate_enter_max",
            "enter_max",
            "exit_min",
            "median_window",
            "enter_consecutive",
            "exit_consecutive",
        ),
    )
    threshold_names = (
        "immediate_enter_max",
        "enter_max",
        "exit_min",
    )
    for name in threshold_names:
        _integer(
            gate[name],
            "calibration.infrared_proximity.obstacle_gate.{}".format(
                name
            ),
            minimum=0,
            maximum=100,
        )
    if not (
        gate["immediate_enter_max"]
        <= gate["enter_max"]
        < gate["exit_min"]
    ):
        raise ValueError("Infrared obstacle gate thresholds are invalid")
    _integer(
        gate["median_window"],
        "calibration.infrared_proximity.obstacle_gate.median_window",
        maximum=MAX_IR_MEDIAN_WINDOW,
    )
    if gate["median_window"] % 2 == 0:
        raise ValueError(
            "Infrared obstacle gate median window must be odd"
        )
    for name in ("enter_consecutive", "exit_consecutive"):
        _integer(
            gate[name],
            "calibration.infrared_proximity.obstacle_gate.{}".format(
                name
            ),
            maximum=MAX_IR_CONSECUTIVE_DECISIONS,
        )

    zones = _require_exact_keys(
        zones,
        "calibration.infrared_proximity.zones",
        ("strong_return_max", "near_return_max", "mid_return_max"),
    )
    for name in zones:
        _integer(
            zones[name],
            "calibration.infrared_proximity.zones.{}".format(name),
            minimum=0,
            maximum=100,
        )
    if not (
        zones["strong_return_max"]
        < zones["near_return_max"]
        < zones["mid_return_max"]
    ):
        raise ValueError("Infrared proximity zones are invalid")


def validate_robot_config(config):
    config = _require_exact_keys(
        config, "configuration", TOP_LEVEL_KEYS
    )
    _validate_finite_tree(config, "configuration")
    if not _is_int(config["schema_version"]):
        raise ValueError("schema_version must be an integer")
    if config["schema_version"] != SCHEMA_VERSION:
        raise ValueError(
            "Unsupported schema_version {}".format(
                config["schema_version"]
            )
        )
    _safe_string(config["robot_id"], "robot_id")
    _safe_string(config["controller_id"], "controller_id")
    limits = _validate_limits(config)
    motors = _validate_motors(config)
    sensors = _validate_sensors(config)
    drive_roles = _validate_drive_geometry(config, motors)
    _validate_agent_api(config, motors, drive_roles)
    _validate_ir_calibration(config, sensors)

    for role, details in motors.items():
        profile = details["limit_profile"]
        if profile not in limits:
            raise ValueError(
                "Motor role {!r} references a missing limit profile".format(
                    role
                )
            )
    return config


def load_robot_config(path):
    with io.open(path, "r", encoding="utf-8") as handle:
        config = json.load(
            handle,
            object_pairs_hook=_strict_object,
            parse_constant=_reject_constant,
        )
    return validate_robot_config(config)
