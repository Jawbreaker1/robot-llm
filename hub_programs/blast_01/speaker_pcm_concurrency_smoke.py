"""Hardware-only proof that BLAST can drive during sampled playback."""

from pybricks.hubs import InventorHub
from pybricks.parameters import Stop
from pybricks.pupdevices import Motor
from pybricks.tools import wait

from wiring import (
    HUB_FRONT_SIDE,
    HUB_TOP_SIDE,
    LEFT_DRIVE_DIRECTION,
    LEFT_DRIVE_PORT,
    RIGHT_DRIVE_DIRECTION,
    RIGHT_DRIVE_PORT,
)


hub = InventorHub(top_side=HUB_TOP_SIDE, front_side=HUB_FRONT_SIDE)
left = Motor(LEFT_DRIVE_PORT, LEFT_DRIVE_DIRECTION, reset_angle=False)
right = Motor(RIGHT_DRIVE_PORT, RIGHT_DRIVE_DIRECTION, reset_angle=False)

# Guard the raw u16le interpretation used by the staged BLE upload.
raw = bytearray(b"\x00\x80\xff\xff")
decoded = (
    raw[0] | raw[1] << 8,
    raw[2] | raw[3] << 8,
)
print("decode", decoded == (32768, 65535))

# Four samples repeated 4,000 times are exactly two seconds at 8 kHz.
samples = bytearray(b"\x00\x80\xff\xbf\x00\x80\x00\x40" * 4000)
try:
    hub.speaker.play_samples(samples, sample_rate=8000, wait=False)
    right.run_angle(90, 90, then=Stop.BRAKE, wait=False)
    left.run_angle(90, 90, then=Stop.BRAKE, wait=False)
    overlap = not hub.speaker.done() and not left.done() and not right.done()
    while not hub.speaker.done() or not left.done() or not right.done():
        wait(10)
    print("concurrent", overlap)
    print("completed", hub.speaker.done())

    hub.speaker.play_samples(samples, sample_rate=8000, wait=False)
    hub.speaker.stop()
    hub.speaker.stop()
    print("cancelled", hub.speaker.done())

    hub.speaker.beep(440, 20)
    hub.speaker.play_notes(["C4/16"], tempo=120)
    print("legacy", "ok")
finally:
    left.brake()
    right.brake()
    hub.speaker.stop()
