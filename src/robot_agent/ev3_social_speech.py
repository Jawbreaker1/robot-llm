"""EV3 conversation gestures using the existing, encoder-checked arm command.

No navigation policy or drive commands live here. The LEGO saw is a prop;
the model selects a gesture name, never motor parameters.
"""

import json
import logging
import subprocess
import time

from .host_piper_speech import _communicate_cancellable
from .shadow_cli import _validate_target
from .social_expression import EV3_GESTURES, valid_social_expression

LOGGER = logging.getLogger(__name__)
# Operator-approved full speed, verified against outA's max_speed attribute.
SAW_SPEED_DPS = 1560
SAW_DURATIONS_MS = {"saw_flourish": 1000, "saw_celebrate": 2000}


class EV3SocialSpeaker:
    def __init__(self, speaker, target, controller_lease, *, gesture_allowed=None,
                 process_factory=subprocess.Popen):
        self.speaker = speaker
        self.target = _validate_target(target)
        self.lease = controller_lease
        self.gesture_allowed = gesture_allowed
        self.process_factory = process_factory

    def __call__(self, text, locale, cancel_event, *, expression=None):
        receipt = self.speaker(text, locale, cancel_event)
        # Speech completes normally even if an accessory is unavailable.
        if receipt is None or cancel_event.is_set() or expression is None:
            return receipt
        if not valid_social_expression(expression, gestures=EV3_GESTURES, faces=("neutral",)):
            LOGGER.warning("EV3 rejected unsupported dialogue expression")
            return receipt
        duration = SAW_DURATIONS_MS.get(expression["gesture"])
        if duration is None or not self.lease.acquire(blocking=False):
            return receipt
        try:
            if not cancel_event.is_set() and (self.gesture_allowed is None or self.gesture_allowed()):
                self._spin(duration, cancel_event)
        except Exception:
            LOGGER.warning("EV3 dialogue saw gesture failed", exc_info=True)
        finally:
            self.lease.release()
        return receipt

    def _spin(self, duration, cancel_event):
        process = self.process_factory([
            "ssh", "-T", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5",
            "-o", "StrictHostKeyChecking=yes", self.target,
            "python3", "/home/robot/robot-llm/ev3/robot_cli.py", "motor-test",
            "--role", "arm", "--speed-dps", str(SAW_SPEED_DPS), "--duration-ms", str(duration),
            "--acknowledge-physical-motion",
        ], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        # A disconnect/cancel cannot leave the arm spinning: the existing HAL
        # sets a kernel-enforced 1–2 second duration and brakes afterwards.
        # Python startup measured ~10 s on this EV3; one gesture is one command.
        output = _communicate_cancellable(process, b"", cancel_event, 30.0, time.monotonic)
        if output is None:
            return False
        stdout, _stderr = output
        if process.returncode != 0 or len(stdout) > 8192:
            raise RuntimeError("EV3 arm command failed")
        result = json.loads(stdout)
        if (result.get("status") != "completed" or result.get("role") != "arm"
                or result.get("port") != "outA" or result.get("speed_dps") != SAW_SPEED_DPS
                or result.get("duration_ms") != duration
                or result.get("verification", {}).get("passed") is not True):
            raise RuntimeError("EV3 arm receipt was not verified")
        LOGGER.info("EV3 saw spin completed duration_ms=%s delta=%s", duration, result.get("position_delta"))
        return True
