"""Play a short sampled waveform through BLAST's hub speaker."""

from pybricks.hubs import InventorHub
from pybricks.tools import wait


SAMPLE_RATE_HZ = 8000

hub = InventorHub()

# 250 Hz triangle wave, 200 ms, unsigned 16-bit PCM for the hub DAC.
cycle = (
    32768, 36863, 40959, 45055, 49151, 53247, 57343, 61439,
    65535, 61439, 57343, 53247, 49151, 45055, 40959, 36863,
    32768, 28672, 24576, 20480, 16384, 12288, 8192, 4096,
    0, 4096, 8192, 12288, 16384, 20480, 24576, 28672,
)
samples = bytearray()
for value in cycle * 50:
    samples.append(value & 0xff)
    samples.append(value >> 8)

hub.speaker.play_samples(
    samples,
    sample_rate=SAMPLE_RATE_HZ,
    wait=False,
)
if hub.speaker.done():
    raise RuntimeError("sampled playback did not start")
while not hub.speaker.done():
    wait(10)
hub.speaker.stop()
print("speaker samples completed asynchronously")
