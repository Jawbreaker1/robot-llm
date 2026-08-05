"""Physically verified port and orientation mapping for LEGO's BLAST build."""

from pybricks.parameters import Axis, Direction, Port


HUB_TOP_SIDE = Axis.X
HUB_FRONT_SIDE = -Axis.Y

RIGHT_DRIVE_PORT = Port.A
RIGHT_DRIVE_DIRECTION = Direction.CLOCKWISE

CLAW_PORT = Port.B

LEFT_DRIVE_PORT = Port.C
LEFT_DRIVE_DIRECTION = Direction.COUNTERCLOCKWISE

BODY_PORT = Port.D
COLOR_SENSOR_PORT = Port.E
ULTRASONIC_SENSOR_PORT = Port.F
