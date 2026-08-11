"""Persistent bounded controller runtime for BLAST-01."""

import json

import micropython
import pybricks
from pybricks.hubs import InventorHub
from pybricks.parameters import Stop
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
SAMPLED_AUDIO_CAPABILITY = "sampled_audio_v2"
SAMPLED_AUDIO_SAMPLE_RATE_HZ = 8000
SAMPLED_AUDIO_ENCODING = "u16le"
SAMPLED_AUDIO_MAX_BYTES = 32000
SAMPLED_AUDIO_MAX_FRAGMENT_BYTES = 252
SAMPLED_AUDIO_RAW_IDLE_TIMEOUT_MS = 5000
DRIVE_PULSE_SPEED_DPS = 120
DRIVE_PULSE_ANGLE_DEG = 90
TURN_PULSE_SPEED_DPS = 180
TURN_PULSE_ANGLE_DEG = 45
CLAW_PULSE_SPEED_DPS = 180
CLAW_PULSE_DURATION_MS = 500
BODY_PULSE_SPEED_DPS = 120
BODY_PULSE_DURATION_MS = 900

hub = InventorHub(
    top_side=HUB_TOP_SIDE,
    front_side=HUB_FRONT_SIDE,
)
clock = StopWatch()

# reset_angle=False preserves positions across runtime deployments. Motion
# operations below are fixed, bounded, and finish with braking.
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
color_sensor = ColorSensor(COLOR_SENSOR_PORT)
ultrasonic_sensor = UltrasonicSensor(ULTRASONIC_SENSOR_PORT)
incoming = poll()
incoming.register(stdin)
sampled_audio_supported = all(
    hasattr(hub.speaker, name)
    for name in ("play_samples", "done", "stop")
)
sampled_audio_transfer = None


class SampledAudioTransportError(Exception):
    pass


def emit(value):
    print(json.dumps(value))


def poll_sampled_audio():
    """Release the DMA buffer as soon as one-shot playback is done."""

    if sampled_audio_supported:
        hub.speaker.done()


def read_line():
    data = bytearray()
    too_large = False
    while True:
        while not incoming.poll(0):
            poll_sampled_audio()
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


def read_exact(size):
    data = bytearray()
    last_data_at_ms = clock.time()
    while len(data) < size:
        while not incoming.poll(0):
            if clock.time() - last_data_at_ms >= (
                SAMPLED_AUDIO_RAW_IDLE_TIMEOUT_MS
            ):
                raise SampledAudioTransportError(
                    "sampled audio payload timed out"
                )
            wait(10)
        # Read only the byte proven available by poll(). A larger blocking
        # read would prevent the no-progress timeout from observing a
        # truncated payload after its first byte.
        chunk = stdin.buffer.read(1)
        if chunk:
            data.extend(chunk)
            last_data_at_ms = clock.time()
    return data


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
        raise ValueError("sample_rate_hz must be 8000")
    if arguments.get("encoding") != SAMPLED_AUDIO_ENCODING:
        raise ValueError("encoding must be u16le")


def begin_pcm(request_id, arguments):
    global sampled_audio_transfer

    validate_pcm_format(arguments)
    byte_count = arguments.get("byte_count")
    if (
        not isinstance(byte_count, int)
        or isinstance(byte_count, bool)
        or byte_count < 2
        or byte_count > SAMPLED_AUDIO_MAX_BYTES
        or byte_count % 2
    ):
        raise ValueError("byte_count must be an even value from 2 to 32000")
    if not all(motor.done() for motor in motors.values()):
        raise ValueError("motors must be idle before sampled audio")
    if not sampled_audio_supported or not hub.speaker.done():
        raise ValueError("speaker must be idle before sampled audio")
    sampled_audio_transfer = {
        "transfer_id": request_id,
        "byte_count": byte_count,
        "received_bytes": 0,
        "payload": bytearray(byte_count),
    }
    return {
        "transfer_id": request_id,
        "byte_count": byte_count,
        "max_fragment_bytes": SAMPLED_AUDIO_MAX_FRAGMENT_BYTES,
    }


def receive_pcm_fragment(request_id, arguments):
    transfer = sampled_audio_transfer
    transfer_id = arguments.get("transfer_id")
    offset = arguments.get("offset")
    byte_count = arguments.get("byte_count")
    if (
        not isinstance(transfer, dict)
        or transfer_id != transfer.get("transfer_id")
        or offset != transfer.get("received_bytes")
        or not isinstance(byte_count, int)
        or isinstance(byte_count, bool)
        or byte_count < 2
        or byte_count > SAMPLED_AUDIO_MAX_FRAGMENT_BYTES
        or byte_count % 2
        or offset + byte_count > transfer.get("byte_count")
    ):
        raise ValueError("sampled audio fragment is invalid")
    if not all(motor.done() for motor in motors.values()):
        raise ValueError("motors must be idle during sampled audio upload")

    # Pybricks normally treats raw 0x03 on stdin as Ctrl-C. PCM may contain
    # any byte value, so suspend interception before inviting the host to send
    # raw data, then restore normal emergency interruption immediately after.
    micropython.kbd_intr(-1)
    try:
        emit(
            sampled_audio_response(
                request_id,
                "ready",
                {
                    "transfer_id": transfer_id,
                    "offset": offset,
                    "byte_count": byte_count,
                },
            )
        )
        payload = read_exact(byte_count)
    finally:
        micropython.kbd_intr(3)
    transfer["payload"][offset:offset + byte_count] = payload
    transfer["received_bytes"] += byte_count
    return {
        "transfer_id": transfer_id,
        "offset": offset,
        "byte_count": byte_count,
        "received_bytes": transfer["received_bytes"],
    }


def start_pcm(arguments):
    global sampled_audio_transfer

    transfer = sampled_audio_transfer
    transfer_id = arguments.get("transfer_id")
    if (
        not isinstance(transfer, dict)
        or transfer_id != transfer.get("transfer_id")
        or transfer.get("received_bytes") != transfer.get("byte_count")
    ):
        raise ValueError("sampled audio upload is incomplete")
    if not all(motor.done() for motor in motors.values()):
        raise ValueError("motors must be idle before sampled audio starts")
    if not sampled_audio_supported or not hub.speaker.done():
        raise ValueError("speaker must be idle before sampled audio starts")

    byte_count = transfer["byte_count"]
    hub.speaker.play_samples(
        transfer["payload"],
        sample_rate=SAMPLED_AUDIO_SAMPLE_RATE_HZ,
        wait=False,
    )
    sampled_audio_transfer = None
    return {
        "transfer_id": transfer_id,
        "byte_count": byte_count,
        "sample_rate_hz": SAMPLED_AUDIO_SAMPLE_RATE_HZ,
        "encoding": SAMPLED_AUDIO_ENCODING,
        "duration_ms": (
            (byte_count // 2) * 1000
            + SAMPLED_AUDIO_SAMPLE_RATE_HZ
            - 1
        ) // SAMPLED_AUDIO_SAMPLE_RATE_HZ,
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


def turn_pulse(direction):
    if direction not in ("left", "right"):
        raise ValueError("direction must be left or right")
    if (
        not motors["right_drive"].done()
        or not motors["left_drive"].done()
    ):
        raise ValueError("drive motors are busy")

    right_angle = (
        TURN_PULSE_ANGLE_DEG
        if direction == "left"
        else -TURN_PULSE_ANGLE_DEG
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
        "wheel_angle_deg": TURN_PULSE_ANGLE_DEG,
        "before_angles_deg": before,
    }


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
                    "max_fragment_bytes": (
                        SAMPLED_AUDIO_MAX_FRAGMENT_BYTES
                    ),
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
        poll_sampled_audio()
        line = read_line()
        request = json.loads(line)
        request_id = request["id"]
        operation = request["op"]
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
        elif operation == "claw_pulse":
            arguments = request.get("args", {})
            result = claw_pulse(arguments.get("direction"))
        elif operation == "body_pulse":
            arguments = request.get("args", {})
            result = body_pulse(arguments.get("direction"))
        elif operation == "play_pcm":
            arguments = request.get("args", {})
            phase = arguments.get("phase")
            if phase == "begin":
                result = begin_pcm(request_id, arguments)
                emit(sampled_audio_response(request_id, "begun", result))
            elif phase == "fragment":
                result = receive_pcm_fragment(request_id, arguments)
                emit(sampled_audio_response(request_id, "received", result))
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
            }
        )
        if isinstance(error, SampledAudioTransportError):
            stop_all()
            break

# Give the final stdout frame time to leave over BLE.
wait(100)
