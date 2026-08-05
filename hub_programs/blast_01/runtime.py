"""Persistent, motion-free observation runtime for BLAST-01."""

import json

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
DRIVE_PULSE_SPEED_DPS = 240
DRIVE_PULSE_ANGLE_DEG = 90

hub = InventorHub(
    top_side=HUB_TOP_SIDE,
    front_side=HUB_FRONT_SIDE,
)
clock = StopWatch()

# reset_angle=False preserves positions. This runtime never calls a motor
# command; the objects exist only so observe can read angle().
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


def emit(value):
    print(json.dumps(value))


def read_line():
    data = bytearray()
    too_large = False
    while True:
        while not incoming.poll(0):
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


def observation():
    imu_ready = hub.imu.ready()
    imu = {
        "ready": imu_ready,
        "stationary": hub.imu.stationary(),
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
        "motion_active": not (
            motors["right_drive"].done()
            and motors["left_drive"].done()
        ),
        "color": str(color_sensor.color()),
        "distance_mm": ultrasonic_sensor.distance(),
    }


def stop_all():
    for motor in motors.values():
        motor.brake()


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


wait(500)
emit(
    {
        "type": "ready",
        "protocol_version": PROTOCOL_VERSION,
        "motion_enabled": False,
        "robot_id": "blast-01",
        "controller_id": "blast-01.hub",
        "firmware": list(pybricks.version),
    }
)

while True:
    request = None
    try:
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

# Give the final stdout frame time to leave over BLE.
wait(100)
