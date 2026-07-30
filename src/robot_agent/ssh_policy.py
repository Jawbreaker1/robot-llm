"""Shared SSH policy for bounded, motion-free EV3 commands.

The physical motor supervisor deliberately owns a separate, explicit
foreground SSH process.  Connection multiplexing here is only for short
inventory, sensor, speech, and setup commands where avoiding a fresh key
exchange materially improves operator latency.
"""

from typing import List


SSH_CONTROL_PATH = "~/.ssh/robot-llm-%C"
SSH_CONTROL_PERSIST_SECONDS = 60


def motion_free_ssh_options(connect_timeout_seconds: int) -> List[str]:
    """Return strict host verification plus short-lived connection reuse."""

    return [
        "-o",
        "BatchMode=yes",
        "-o",
        "ConnectTimeout={}".format(connect_timeout_seconds),
        "-o",
        "StrictHostKeyChecking=yes",
        "-o",
        "ControlMaster=auto",
        "-o",
        "ControlPath={}".format(SSH_CONTROL_PATH),
        "-o",
        "ControlPersist={}".format(SSH_CONTROL_PERSIST_SECONDS),
    ]
