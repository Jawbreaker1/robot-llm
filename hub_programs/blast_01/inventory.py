"""Report BLAST-01 hardware without commanding any attached motor."""

import pybricks
from pybricks.hubs import InventorHub
from pybricks.iodevices import PUPDevice
from pybricks.parameters import Axis, Port
from pybricks.tools import wait


DEVICE_NAMES = {
    48: "SPIKE Medium Angular Motor",
    49: "SPIKE Large Angular Motor",
    61: "SPIKE Color Sensor",
    62: "SPIKE Ultrasonic Sensor",
    63: "SPIKE Force Sensor",
    65: "SPIKE Small Angular Motor",
    75: "Technic Medium Angular Motor",
    76: "Technic Large Angular Motor",
}

# Orientation of the Inventor Hub in LEGO's BLAST build.
hub = InventorHub(top_side=Axis.X, front_side=-Axis.Y)

# Give UART devices and the stationary IMU time to settle.
wait(1000)

print("BLAST_INVENTORY_BEGIN")
print("PYBRICKS_VERSION", pybricks.version)
print("HUB", hub.system.info())
print("BATTERY_MV", hub.battery.voltage())
print("BATTERY_MA", hub.battery.current())

imu_ready = hub.imu.ready()
print("IMU_READY", imu_ready)
print("IMU_STATIONARY", hub.imu.stationary())
print("IMU_UP_RAW", hub.imu.up(False))
if imu_ready:
    print("IMU_TILT", hub.imu.tilt())
    # Heading is relative to program start, not an absolute compass bearing.
    print("IMU_HEADING", hub.imu.heading())

for label, port in (
    ("A", Port.A),
    ("B", Port.B),
    ("C", Port.C),
    ("D", Port.D),
    ("E", Port.E),
    ("F", Port.F),
):
    try:
        info = PUPDevice(port).info()
    except OSError as error:
        print("PORT", label, "EMPTY_OR_UNAVAILABLE", error.args)
        continue

    device_id = info["id"]
    print("PORT", label, device_id, DEVICE_NAMES.get(device_id, "UNKNOWN"))

print("BLAST_INVENTORY_OK")

# Give Bluetooth stdout time to flush before the program exits.
wait(500)
