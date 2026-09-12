"""Persistent bounded controller runtime for BLAST-01."""

import gc
import json

import pybricks
from pybricks.hubs import InventorHub
from pybricks.messaging import AppData
from pybricks.parameters import Color, Side, Stop
from pybricks.pupdevices import ColorSensor, Motor, UltrasonicSensor
from pybricks.tools import StopWatch, wait
from uselect import poll
from usys import stdin

from wiring import (
    BODY_PORT,
    CLAW_PORT,
    COLOR_SENSOR_PORT,
    HUB_FRONT_SIDE,
    HUB_TOP_SIDE,
    LEFT_DRIVE_DIRECTION,
    LEFT_DRIVE_PORT,
    RIGHT_DRIVE_DIRECTION,
    RIGHT_DRIVE_PORT,
    ULTRASONIC_SENSOR_PORT,
)


PROTOCOL_VERSION = 1
MAX_INPUT_CHARS = 512
SAMPLED_AUDIO_CAPABILITY = "sampled_audio_v5"
SAMPLED_AUDIO_SAMPLE_RATE_HZ = 16000
SAMPLED_AUDIO_ENCODING = "ima_adpcm4_mono_stream_v1"
SAMPLED_AUDIO_MAX_SAMPLES = 128000
SAMPLED_AUDIO_HEADER_BYTES = 7
SAMPLED_AUDIO_MAX_BYTES = (
    SAMPLED_AUDIO_HEADER_BYTES + SAMPLED_AUDIO_MAX_SAMPLES // 2
)
SAMPLED_AUDIO_DMA_CHUNK_SAMPLES = 256
SAMPLED_AUDIO_DMA_CHUNK_DURATION_MS = 16
SAMPLED_AUDIO_TRANSPORT = "app_data_v1"
SAMPLED_AUDIO_CHECKSUM = "fletcher16"
DRIVE_PULSE_SPEED_DPS = 120
DRIVE_PULSE_ANGLE_DEG = 90
TURN_PULSE_SPEED_DPS = 180
TURN_PULSE_ANGLE_DEG = 45
SCAN_TURN_PULSE_ANGLE_DEG = 45
SCAN_TRIM_PULSE_ANGLE_DEG = 15
CLAW_PULSE_SPEED_DPS = 180
CLAW_PULSE_DURATION_MS = 500
BODY_PULSE_SPEED_DPS = 120
BODY_PULSE_DURATION_MS = 900
MOTOR_SETTLE_MS = 150
FACE_PATTERNS = {
    "neutral": ("00000", "11011", "11011", "11011", "00000"),
    "happy": ("00000", "01010", "10101", "00000", "00000"),
    "frustrated": ("10001", "11011", "01010", "00000", "00000"),
    "curious": ("00011", "11011", "11011", "11000", "00000"),
    "surprised": ("11011", "11011", "11011", "11011", "00000"),
    # The monochrome matrix reads better as one large frown than a white X.
    "angry": ("00000", "01110", "10001", "10001", "00000"),
}
# A blink and small eye movements, not a scan or a change of emotion.
IDLE_EYE_PATTERNS = (
    FACE_PATTERNS["neutral"],
    ("00000", "00000", "11011", "00000", "00000"),
    ("00000", "10010", "10010", "10010", "00000"),
    ("00000", "01001", "01001", "01001", "00000"),
)

hub = InventorHub(
    top_side=HUB_TOP_SIDE,
    front_side=HUB_FRONT_SIDE,
)
clock = StopWatch()

# reset_angle=False preserves positions across runtime deployments. Motion
# operations below are fixed and bounded; settled stops are then released
# so the IMU can recalibrate while BLAST is at rest.
motors = {
    "right_drive": Motor(
        RIGHT_DRIVE_PORT,
        RIGHT_DRIVE_DIRECTION,
        reset_angle=False,
    ),
    "claw": Motor(CLAW_PORT, reset_angle=False),
    "left_drive": Motor(
        LEFT_DRIVE_PORT,
        LEFT_DRIVE_DIRECTION,
        reset_angle=False,
    ),
    "body": Motor(BODY_PORT, reset_angle=False),
}
# The default 8-degree integral dead zone leaves the geared sensor arm short
# of its reference. Let the motor's own controller finish the return pose.
motors["body"].control.pid(integral_deadzone=1)
motors["body"].control.target_tolerances(position=1)
color_sensor = ColorSensor(COLOR_SENSOR_PORT)
ultrasonic_sensor = UltrasonicSensor(ULTRASONIC_SENSOR_PORT)
incoming = poll()
incoming.register(stdin)
sampled_audio_supported = all(
    hasattr(hub.speaker, name)
    for name in ("play_adpcm", "done", "stop")
)
sampled_audio_app_data = AppData([(0, SAMPLED_AUDIO_MAX_BYTES)])
sampled_audio_transfer = None
motor_release_at_ms = None


def emit(value):
    print(json.dumps(value))


def poll_background_tasks():
    """Release completed audio and settled motors, allowing IMU calibration."""
    global motor_release_at_ms

    if sampled_audio_supported:
        hub.speaker.done()
    if motor_release_at_ms is None:
        return
    if not all(motor.done() for motor in motors.values()):
        motor_release_at_ms = clock.time() + MOTOR_SETTLE_MS
    elif clock.time() >= motor_release_at_ms:
        # Pybricks calibrates only with ALL motors coasting, not BRAKE/HOLD.
        # Keep the commanded stop until motion has finished and settled.
        for motor in motors.values():
            motor.stop()
        motor_release_at_ms = None


def read_line():
    data = bytearray()
    too_large = False
    while True:
        while not incoming.poll(0):
            poll_background_tasks()
            wait(10)
        character = stdin.buffer.read(1)
        if character == b"\n":
            if too_large:
                raise ValueError("request is too large")
            return str(data, "utf-8")
        if character != b"\r":
            if len(data) < MAX_INPUT_CHARS:
                data.extend(character)
            else:
                too_large = True


def response(request_id, operation, result):
    return {
        "id": request_id,
        "ok": True,
        "op": operation,
        "result": result,
    }


def sampled_audio_response(request_id, phase, result):
    value = response(request_id, "play_pcm", result)
    value["phase"] = phase
    return value


def observation():
    imu_ready = hub.imu.ready()
    imu = {
        "ready": imu_ready,
        "stationary": hub.imu.stationary(),
        # Static front/back lean is useful even while the calibrated IMU is
        # not ready. Keep it explicitly separate from calibrated tilt below.
        "raw_tilt_deg": list(hub.imu.tilt(False)),
    }
    if imu_ready:
        imu["heading_deg"] = hub.imu.heading()
        imu["tilt_deg"] = list(hub.imu.tilt())

    return {
        "observed_at_ms": clock.time(),
        "battery": {
            "voltage_mv": hub.battery.voltage(),
            "current_ma": hub.battery.current(),
        },
        "imu": imu,
        "motor_angles_deg": {
            name: motor.angle() for name, motor in motors.items()
        },
        "motion_active": not all(
            motor.done() for motor in motors.values()
        ),
        "color": str(color_sensor.color()),
        "distance_mm": ultrasonic_sensor.distance(),
    }


def stop_all():
    global sampled_audio_transfer

    for motor in motors.values():
        motor.brake()
    if sampled_audio_supported:
        hub.speaker.stop()
    sampled_audio_transfer = None


def validate_pcm_format(arguments):
    if arguments.get("sample_rate_hz") != SAMPLED_AUDIO_SAMPLE_RATE_HZ:
        raise ValueError("sample_rate_hz must be 16000")
    if arguments.get("encoding") != SAMPLED_AUDIO_ENCODING:
        raise ValueError("encoding must be ima_adpcm4_mono_stream_v1")


def begin_pcm(request_id, arguments):
    global sampled_audio_transfer

    sampled_audio_transfer = None
    validate_pcm_format(arguments)
    byte_count = arguments.get("byte_count")
    sample_count = arguments.get("sample_count")
    checksum = arguments.get("fletcher16")
    if (
        not isinstance(byte_count, int)
        or isinstance(byte_count, bool)
        or byte_count < SAMPLED_AUDIO_HEADER_BYTES
        or byte_count > SAMPLED_AUDIO_MAX_BYTES
    ):
        raise ValueError("sampled audio byte_count is invalid")
    if (
        not isinstance(sample_count, int)
        or isinstance(sample_count, bool)
        or sample_count < 1
        or sample_count > SAMPLED_AUDIO_MAX_SAMPLES
        or byte_count != SAMPLED_AUDIO_HEADER_BYTES + sample_count // 2
    ):
        raise ValueError("sampled audio sample_count is invalid")
    if (
        not isinstance(checksum, int)
        or isinstance(checksum, bool)
        or checksum < 0
        or checksum > 0xFFFF
    ):
        raise ValueError("fletcher16 must be a uint16 integer")
    if not sampled_audio_supported or not hub.speaker.done():
        raise ValueError("speaker must be idle before sampled audio")
    sampled_audio_transfer = {
        "transfer_id": request_id,
        "sample_rate_hz": SAMPLED_AUDIO_SAMPLE_RATE_HZ,
        "encoding": SAMPLED_AUDIO_ENCODING,
        "sample_count": sample_count,
        "byte_count": byte_count,
        "fletcher16": checksum,
    }
    return dict(sampled_audio_transfer)


def start_pcm(arguments):
    global sampled_audio_transfer

    transfer = sampled_audio_transfer
    transfer_id = arguments.get("transfer_id")
    if (
        not isinstance(transfer, dict)
        or transfer_id != transfer.get("transfer_id")
    ):
        raise ValueError("sampled audio transfer is invalid")
    # A recognized start consumes the transfer even when its remaining
    # metadata is malformed or decoding runs out of memory.
    sampled_audio_transfer = None
    for name in (
        "sample_rate_hz",
        "encoding",
        "sample_count",
        "byte_count",
        "fletcher16",
    ):
        if arguments.get(name) != transfer.get(name):
            raise ValueError("sampled audio transfer metadata changed")
    if not sampled_audio_supported or not hub.speaker.done():
        raise ValueError("speaker must be idle before sampled audio starts")

    byte_count = transfer["byte_count"]
    sample_count = transfer["sample_count"]
    checksum = transfer["fletcher16"]
    # done() releases the native DMA root, but MicroPython does not
    # necessarily reclaim the previous maximum-sized snapshot before the
    # next AppData allocation. Collect once at this bounded hand-off so
    # consecutive utterances cannot exhaust an otherwise healthy heap.
    gc.collect()
    payload = sampled_audio_app_data.get_bytes(0)
    hub.speaker.play_adpcm(
        payload,
        byte_count=byte_count,
        sample_count=sample_count,
        fletcher16=checksum,
        sample_rate=SAMPLED_AUDIO_SAMPLE_RATE_HZ,
        wait=False,
    )
    return {
        "transfer_id": transfer_id,
        "byte_count": byte_count,
        "sample_count": sample_count,
        "sample_rate_hz": SAMPLED_AUDIO_SAMPLE_RATE_HZ,
        "encoding": SAMPLED_AUDIO_ENCODING,
        "duration_ms": (
            (sample_count + SAMPLED_AUDIO_DMA_CHUNK_SAMPLES - 1)
            // SAMPLED_AUDIO_DMA_CHUNK_SAMPLES
            * SAMPLED_AUDIO_DMA_CHUNK_DURATION_MS
        ),
        "fletcher16": checksum,
    }


def drive_pulse(direction):
    if direction not in ("forward", "reverse"):
        raise ValueError("direction must be forward or reverse")
    if (
        not motors["right_drive"].done()
        or not motors["left_drive"].done()
    ):
        raise ValueError("drive motors are busy")

    angle = (
        DRIVE_PULSE_ANGLE_DEG
        if direction == "forward"
        else -DRIVE_PULSE_ANGLE_DEG
    )
    before = {
        "right_drive": motors["right_drive"].angle(),
        "left_drive": motors["left_drive"].angle(),
    }
    motors["right_drive"].run_angle(
        DRIVE_PULSE_SPEED_DPS,
        angle,
        then=Stop.BRAKE,
        wait=False,
    )
    try:
        motors["left_drive"].run_angle(
            DRIVE_PULSE_SPEED_DPS,
            angle,
            then=Stop.BRAKE,
            wait=False,
        )
    except Exception:
        motors["right_drive"].brake()
        raise
    return {
        "accepted": True,
        "direction": direction,
        "speed_dps": DRIVE_PULSE_SPEED_DPS,
        "angle_deg": DRIVE_PULSE_ANGLE_DEG,
        "before_angles_deg": before,
    }


def fixed_turn_pulse(direction, wheel_angle_deg):
    if direction not in ("left", "right"):
        raise ValueError("direction must be left or right")
    if (
        not motors["right_drive"].done()
        or not motors["left_drive"].done()
    ):
        raise ValueError("drive motors are busy")

    right_angle = (
        wheel_angle_deg
        if direction == "left"
        else -wheel_angle_deg
    )
    left_angle = -right_angle
    before = {
        "right_drive": motors["right_drive"].angle(),
        "left_drive": motors["left_drive"].angle(),
    }
    motors["right_drive"].run_angle(
        TURN_PULSE_SPEED_DPS,
        right_angle,
        then=Stop.BRAKE,
        wait=False,
    )
    try:
        motors["left_drive"].run_angle(
            TURN_PULSE_SPEED_DPS,
            left_angle,
            then=Stop.BRAKE,
            wait=False,
        )
    except Exception:
        motors["right_drive"].brake()
        raise
    return {
        "accepted": True,
        "direction": direction,
        "speed_dps": TURN_PULSE_SPEED_DPS,
        "wheel_angle_deg": wheel_angle_deg,
        "before_angles_deg": before,
    }


def turn_pulse(direction):
    return fixed_turn_pulse(direction, TURN_PULSE_ANGLE_DEG)


def turn_trim_pulse(direction):
    return fixed_turn_pulse(direction, 15)


def scan_turn_pulse(direction):
    return fixed_turn_pulse(direction, SCAN_TURN_PULSE_ANGLE_DEG)


def scan_trim_pulse(direction):
    return fixed_turn_pulse(direction, SCAN_TRIM_PULSE_ANGLE_DEG)


def set_pose(role, target_angle_deg):
    """Position one accessory motor without resetting any encoder or IMU."""
    if role not in ("body", "claw"):
        raise ValueError("pose motor must be body or claw")
    if type(target_angle_deg) is not int or not -1000000 <= target_angle_deg <= 1000000:
        raise ValueError("pose target must be an integer angle")
    if not all(motor.done() for motor in motors.values()):
        raise ValueError("motors must be idle before setting a pose")
    motor = motors[role]
    before = motor.angle()
    # BLAST's geared arm needs ~800 motor degrees for its full excursion.
    # These are motor angles, not arm angles; the claw has a different linkage.
    max_travel = 900 if role == "body" else 180
    if abs(target_angle_deg - before) > max_travel:
        raise ValueError("pose target exceeds accessory travel")
    # Finish the pose, then brake until the normal idle release. HOLD keeps
    # correcting tiny deflections and can become "busy" again between poses.
    motor.run_target(500 if role == "body" else 180, target_angle_deg,
                     then=Stop.BRAKE, wait=False)
    return {
        "accepted": True,
        "motor": role,
        "before_angle_deg": before,
        "target_angle_deg": target_angle_deg,
    }


def show_face(expression):
    if expression not in FACE_PATTERNS and expression != "idle":
        raise ValueError("unknown face expression")
    # The display is mounted sideways in BLAST; do not rotate the IMU frame.
    hub.display.orientation(up=Side.RIGHT)
    hub.display.off()  # Replace any previous animation immediately.
    if expression == "idle":
        frames = [
            [[100 if pixel == "1" else 0 for pixel in row] for row in pattern]
            for pattern in IDLE_EYE_PATTERNS
        ]
        sequence = (frames[0:1] * 24 + frames[1:2] + frames[0:1] * 15
                    + frames[2:3] * 8 + frames[0:1] * 10
                    + frames[3:4] * 8 + frames[0:1] * 10 + frames[1:2])
        hub.display.animate(sequence, interval=150)
    else:
        hub.display.icon([
            [100 if pixel == "1" else 0 for pixel in row]
            for row in FACE_PATTERNS[expression]
        ])
    return {"accepted": True, "expression": expression}


def claw_pulse(direction):
    if direction not in ("open", "close"):
        raise ValueError("direction must be open or close")
    if not motors["claw"].done():
        raise ValueError("claw motor is busy")

    speed = (
        CLAW_PULSE_SPEED_DPS
        if direction == "open"
        else -CLAW_PULSE_SPEED_DPS
    )
    before = motors["claw"].angle()
    motors["claw"].run_time(
        speed,
        CLAW_PULSE_DURATION_MS,
        then=Stop.BRAKE,
        wait=False,
    )
    return {
        "accepted": True,
        "direction": direction,
        "speed_dps": CLAW_PULSE_SPEED_DPS,
        "duration_ms": CLAW_PULSE_DURATION_MS,
        "before_angle_deg": before,
    }


def body_pulse(direction):
    if direction not in ("left", "right"):
        raise ValueError("direction must be left or right")
    if not motors["body"].done():
        raise ValueError("body motor is busy")

    speed = (
        -BODY_PULSE_SPEED_DPS
        if direction == "left"
        else BODY_PULSE_SPEED_DPS
    )
    before = motors["body"].angle()
    motors["body"].run_time(
        speed,
        BODY_PULSE_DURATION_MS,
        then=Stop.BRAKE,
        wait=False,
    )
    return {
        "accepted": True,
        "direction": direction,
        "speed_dps": BODY_PULSE_SPEED_DPS,
        "duration_ms": BODY_PULSE_DURATION_MS,
        "before_angle_deg": before,
    }


wait(500)
# Reserve the matrix for eyes; the button light indicates the active runtime.
show_face("idle")
hub.light.on(Color.GREEN)
emit(
    {
        "type": "ready",
        "protocol_version": PROTOCOL_VERSION,
        "motion_enabled": True,
        "robot_id": "blast-01",
        "controller_id": "blast-01.hub",
        "firmware": list(pybricks.version),
        "capabilities": (
            {
                SAMPLED_AUDIO_CAPABILITY: {
                    "sample_rate_hz": SAMPLED_AUDIO_SAMPLE_RATE_HZ,
                    "encoding": SAMPLED_AUDIO_ENCODING,
                    "max_bytes": SAMPLED_AUDIO_MAX_BYTES,
                    "transport": SAMPLED_AUDIO_TRANSPORT,
                    "checksum": SAMPLED_AUDIO_CHECKSUM,
                }
            }
            if sampled_audio_supported
            else {}
        ),
    }
)

while True:
    request = None
    try:
        poll_background_tasks()
        line = read_line()
        request = json.loads(line)
        request_id = request["id"]
        operation = request["op"]
        if operation in (
            "stop", "drive_pulse", "turn_pulse", "turn_trim_pulse",
            "scan_turn_pulse", "scan_trim_pulse", "claw_pulse",
            "body_pulse", "set_pose",
        ):
            motor_release_at_ms = clock.time() + MOTOR_SETTLE_MS
        if operation == "ping":
            result = {"uptime_ms": clock.time()}
        elif operation == "observe":
            result = observation()
        elif operation == "stop":
            stop_all()
            result = {"stopped": True}
        elif operation == "drive_pulse":
            arguments = request.get("args", {})
            result = drive_pulse(arguments.get("direction"))
        elif operation == "turn_pulse":
            arguments = request.get("args", {})
            result = turn_pulse(arguments.get("direction"))
        elif operation == "turn_trim_pulse":
            arguments = request.get("args", {})
            result = turn_trim_pulse(arguments.get("direction"))
        elif operation == "scan_turn_pulse":
            arguments = request.get("args", {})
            result = scan_turn_pulse(arguments.get("direction"))
        elif operation == "scan_trim_pulse":
            arguments = request.get("args", {})
            result = scan_trim_pulse(arguments.get("direction"))
        elif operation == "claw_pulse":
            arguments = request.get("args", {})
            result = claw_pulse(arguments.get("direction"))
        elif operation == "body_pulse":
            arguments = request.get("args", {})
            result = body_pulse(arguments.get("direction"))
        elif operation == "set_pose":
            arguments = request.get("args", {})
            result = set_pose(arguments.get("motor"), arguments.get("target_angle_deg"))
        elif operation == "show_face":
            arguments = request.get("args", {})
            result = show_face(arguments.get("expression"))
        elif operation == "play_pcm":
            arguments = request.get("args", {})
            phase = arguments.get("phase")
            if phase == "begin":
                result = begin_pcm(request_id, arguments)
                emit(sampled_audio_response(request_id, "begun", result))
            elif phase == "start":
                result = start_pcm(arguments)
                emit(sampled_audio_response(request_id, "started", result))
            else:
                raise ValueError("unsupported sampled audio phase")
            continue
        elif operation == "shutdown":
            stop_all()
            emit(
                response(
                    request_id,
                    operation,
                    {"shutting_down": True},
                )
            )
            break
        else:
            raise ValueError("unsupported operation")
        emit(response(request_id, operation, result))
    except Exception as error:
        emit(
            {
                "id": request.get("id")
                if isinstance(request, dict)
                else None,
                "ok": False,
                "op": request.get("op")
                if isinstance(request, dict)
                else None,
                "error": str(error),
                "error_type": "rejected" if isinstance(error, ValueError) else "runtime",
            }
        )

# Give the final stdout frame time to leave over BLE.
wait(100)
