"""Explicit stationary accessory probe; never commands either drive motor.

Default: inspect only. Use --gesture faces, claw, body or restore after positioning BLAST
with room for its arms. This is a hardware probe, not a model-selected behavior.
"""
import argparse
import asyncio
import json
import threading

from robot_agent.blast_ble_runtime import BlastBLERuntime
from robot_agent.blast_hub_speech import BLAST_PIPER_PROFILE, pcm16_wav_to_blast_adpcm
from robot_agent.blast_navigation_calibration import BLAST_PROVISIONAL_NAVIGATION_CALIBRATION
from robot_agent.host_piper_speech import PiperLoopbackSynthesizer


# Hardware-demo lines only: the production model will choose its own words.
DEMO_LINES = {
    "happy": "That's more like it. A girl needs a little personality!",
    "curious": "Now, what have we got here?",
    "angry": "I'm absolutely furious. Mostly for dramatic effect.",
    "idle": "I'm still here. Just keeping an eye on things.",
}


async def prepare_speech():
    synthesizer = PiperLoopbackSynthesizer(BLAST_PIPER_PROFILE)
    cancel = threading.Event()
    prepared = {}
    try:
        for expression, text in DEMO_LINES.items():
            raw = await asyncio.to_thread(synthesizer.synthesize, text, "en", cancel)
            prepared[expression] = pcm16_wav_to_blast_adpcm(raw)
            print(json.dumps({"speech_prepared": expression, "text": text,
                              "voice": BLAST_PIPER_PROFILE.voice_for_locale("en"),
                              "duration_ms": prepared[expression].duration_ms}), flush=True)
    finally:
        cancel.set()
    return prepared


async def present_face(runtime, expression, audio=None):
    # One BLE owner. Preload, show the face, then start native playback.
    if audio is not None:
        transfer = await runtime.begin_pcm(audio.payload)
        size = transfer["batch_bytes"]
        for offset in range(0, len(audio.payload), size):
            await runtime.write_pcm_batch(offset, audio.payload[offset:offset + size])
    print(json.dumps(await runtime.show_face(expression)), flush=True)
    if audio is None:
        return 0
    receipt = await runtime.start_pcm(
        transfer["transfer_id"], len(audio.payload), transfer["fletcher16"],
    )
    print(json.dumps({"speech_started": expression, "receipt": receipt}), flush=True)
    return receipt["duration_ms"] / 1000


def report(event, observation):
    print(json.dumps({"event": event, "motors": observation["motor_angles_deg"],
                      "distance_mm": observation["distance_mm"],
                      "motion_active": observation["motion_active"]}), flush=True)


async def pose(runtime, role, target):
    await runtime.set_pose(role, target)
    deadline = asyncio.get_running_loop().time() + 4
    while True:
        observed = await runtime.observe()
        if observed["motion_active"] is False:
            await asyncio.sleep(0.25)
            observed = await runtime.observe()
            report(role + "_target_" + str(target), observed)
            return observed
        if asyncio.get_running_loop().time() >= deadline:
            report("unfinished_" + role + "_target_" + str(target), observed)
            await runtime.stop()
            raise RuntimeError("Accessory did not finish; stopped without retrying")
        await asyncio.sleep(0.05)


async def run(args):
    speech = await prepare_speech() if args.speech else {}
    runtime = BlastBLERuntime(hub_name=args.hub_name)
    await asyncio.wait_for(runtime.connect(), timeout=30)
    try:
        before = await runtime.observe()
        report("before", before)
        if before["motion_active"]:
            raise RuntimeError("BLAST must be stationary")
        if args.gesture == "faces":
            for expression in ("neutral", "happy", "frustrated", "curious", "surprised", "angry", "idle", "neutral"):
                duration = await present_face(runtime, expression, speech.get(expression))
                await asyncio.sleep(max(13 if expression == "idle" else 3, duration + 0.5))
                if expression == "idle":
                    report("observe_during_idle_animation", await runtime.observe())
        elif args.gesture == "claw":
            start = (before["motor_angles_deg"]["claw"]
                     if args.claw_closed_angle is None else args.claw_closed_angle)
            neutral = BLAST_PROVISIONAL_NAVIGATION_CALIBRATION.range_sensor_extrinsics.navigation_body_motor_angle_deg
            await runtime.show_face("happy")
            if args.present_claw:
                await pose(runtime, "body", neutral - 600)
            if args.claw_closed_angle is not None:
                await pose(runtime, "claw", start)
            for target in (start + args.claw_offset, start, start + args.claw_offset, start):
                await pose(runtime, "claw", target)
                await asyncio.sleep(0.4)
            if args.present_claw:
                await pose(runtime, "body", neutral)
        elif args.gesture in ("body", "restore"):
            sensor = BLAST_PROVISIONAL_NAVIGATION_CALIBRATION.range_sensor_extrinsics
            neutral = sensor.navigation_body_motor_angle_deg
            await runtime.show_face("neutral" if args.gesture == "restore" else "frustrated")
            targets = (neutral,) if args.gesture == "restore" else (neutral + args.body_offset, neutral)
            for target in targets:
                await pose(runtime, "body", target)
                await asyncio.sleep(3)
        after = await runtime.observe()
        report("after", after)
        # Encoders can move when the operator lifts BLAST to see the display.
        # Report that separately; this probe never sends a drive command.
        print(json.dumps({"drive_encoder_delta_deg": {
            role: after["motor_angles_deg"][role] - before["motor_angles_deg"][role]
            for role in ("left_drive", "right_drive")
        }}), flush=True)
        sensor = BLAST_PROVISIONAL_NAVIGATION_CALIBRATION.range_sensor_extrinsics
        print(json.dumps({"navigation_sensor_pose_matches": sensor.matches_navigation_body_angle(
            after["motor_angles_deg"]["body"])}), flush=True)
        if args.gesture != "none":
            await runtime.show_face("neutral")
    finally:
        await runtime.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--hub-name", default="BLAST-01")
    parser.add_argument("--gesture", choices=("none", "faces", "claw", "body", "restore"), default="none")
    parser.add_argument("--body-offset", type=int, help="Measured body-motor excursion from navigation pose")
    parser.add_argument("--claw-offset", type=int, default=45, help="Claw opening excursion in motor degrees")
    parser.add_argument("--claw-closed-angle", type=int, help="Explicit measured closed reference for a claw already left open")
    parser.add_argument("--present-claw", action="store_true", help="Lift the arms during the claw probe, then restore the sensor pose")
    parser.add_argument("--speech", action="store_true", help="Pair the face demo with BLAST's English Piper voice")
    args = parser.parse_args()
    if args.speech and args.gesture != "faces":
        parser.error("--speech currently accompanies the faces demo only")
    if args.gesture == "body" and (args.body_offset is None or not 0 < abs(args.body_offset) <= 900):
        parser.error("body probe requires an explicit --body-offset between -900 and 900 (nonzero)")
    if not 0 < args.claw_offset <= 180:
        parser.error("claw offset must be between 1 and 180 motor degrees")
    if args.present_claw and args.gesture != "claw":
        parser.error("--present-claw requires --gesture claw")
    asyncio.run(run(args))
