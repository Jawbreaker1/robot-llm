"""Persistent, motion-free observation runtime for BLAST-01."""

import json

import pybricks
from pybricks.hubs import InventorHub
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
        "color": str(color_sensor.color()),
        "distance_mm": ultrasonic_sensor.distance(),
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
        elif operation == "shutdown":
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
