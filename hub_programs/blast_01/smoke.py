"""Motor-free end-to-end smoke test for the BLAST-01 Pybricks hub."""

from pybricks.hubs import InventorHub
from pybricks.tools import wait


hub = InventorHub()
hub.speaker.volume(30)

print("BLAST_CLI_SMOKE_STARTED")
hub.display.text("CLI")
hub.speaker.beep(880, 250)
print("BLAST_CLI_SMOKE_OK")

# Give the Bluetooth stdout channel time to flush before the program exits.
wait(250)
