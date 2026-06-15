#!/usr/bin/env python3
import argparse
import json
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from auto_parking.config import load_config  # noqa: E402
from auto_parking.control.drive import RoverDrive, clip, compute_wheel_speeds  # noqa: E402


STEP_STEER = 0.15
STEP_SPEED = 0.05
UPDATE_INTERVAL = 0.1
DEFAULT_OUTPUT = PROJECT_ROOT / "recordings" / "parking_demo.json"


def parse_args():
    parser = argparse.ArgumentParser(
        description="Record manual rover commands for later replay.",
    )
    parser.add_argument(
        "-o",
        "--output",
        default=str(DEFAULT_OUTPUT),
        help="output JSON path",
    )
    parser.add_argument(
        "--config",
        default=None,
        help="config YAML path; defaults to config.yaml plus config.local.yaml",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=UPDATE_INTERVAL,
        help="record/send interval in seconds",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="record commands without opening the rover serial port",
    )
    return parser.parse_args()


def key_name(key):
    from pynput import keyboard

    try:
        return key.char
    except AttributeError:
        if key == keyboard.Key.space:
            return "space"
    return None


def main():
    args = parse_args()
    from pynput import keyboard

    config = load_config(args.config)
    output_path = Path(args.output).expanduser()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    drive = None if args.dry_run else RoverDrive(config)
    pressed = set()
    steering = 0.0
    speed = 0.0
    samples = []
    start_time = time.monotonic()

    print("Manual parking record started.")
    print("Controls: w/s speed, a/d steering, space stop, q finish and save.")
    if args.dry_run:
        print("DRY-RUN: serial port is not opened.")

    def on_press(key):
        name = key_name(key)
        if name:
            pressed.add(name)

    def on_release(key):
        name = key_name(key)
        if name:
            pressed.discard(name)
            if name == "q":
                return False
        return None

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    try:
        while listener.running:
            loop_start = time.monotonic()

            if "w" in pressed:
                speed += STEP_SPEED
            elif "s" in pressed:
                speed -= STEP_SPEED
            else:
                speed *= 0.88

            if "a" in pressed:
                steering -= STEP_STEER
            elif "d" in pressed:
                steering += STEP_STEER
            else:
                steering *= 0.55

            if "space" in pressed:
                speed = 0.0
                steering = 0.0

            max_speed = float(config["rover"]["max_speed"])
            max_steer = float(config["rover"]["max_steer"])
            speed = clip(speed, max_speed)
            steering = clip(steering, max_steer)
            left, right = compute_wheel_speeds(steering, speed, max_steer, max_speed)
            if drive is not None:
                left, right = drive.send(steering, speed)

            t = loop_start - start_time
            samples.append(
                {
                    "t": round(t, 4),
                    "steering": round(steering, 4),
                    "speed": round(speed, 4),
                    "left": round(left, 4),
                    "right": round(right, 4),
                    "keys": sorted(pressed),
                },
            )
            print(
                f"t={t:6.2f}s speed={speed:+.2f} steering={steering:+.2f} "
                f"L={left:+.2f} R={right:+.2f}",
            )

            sleep_time = max(0.0, args.interval - (time.monotonic() - loop_start))
            time.sleep(sleep_time)
    finally:
        if drive is not None:
            drive.stop()
        listener.stop()

    data = {
        "version": 1,
        "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "interval": args.interval,
        "duration": round(samples[-1]["t"], 4) if samples else 0.0,
        "rover": {
            "max_speed": float(config["rover"]["max_speed"]),
            "max_steer": float(config["rover"]["max_steer"]),
        },
        "samples": samples,
    }
    with output_path.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
        f.write("\n")

    print(f"Saved {len(samples)} samples to {output_path}")


if __name__ == "__main__":
    main()
