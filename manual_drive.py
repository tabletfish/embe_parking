#!/usr/bin/env python3
import sys
import time
from pathlib import Path
import argparse

PROJECT_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from auto_parking.config import load_config  # noqa: E402
from auto_parking.control.drive import RoverDrive, clip  # noqa: E402


STEP_STEER = 0.15
STEP_SPEED = 0.05
UPDATE_INTERVAL = 0.1
DEFAULT_COMMAND_MAX_SPEED = 0.5
SPEED_DECAY = 0.85
STEER_DECAY = 0.60


def parse_args():
    parser = argparse.ArgumentParser(description="Manual keyboard rover drive.")
    parser.add_argument(
        "--step-speed",
        type=float,
        default=STEP_SPEED,
        help="speed change per update while w/s is pressed",
    )
    parser.add_argument(
        "--step-steer",
        type=float,
        default=STEP_STEER,
        help="steering change per update while a/d is pressed",
    )
    parser.add_argument(
        "--max-command-speed",
        type=float,
        default=DEFAULT_COMMAND_MAX_SPEED,
        help="manual-drive speed limit",
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

    config = load_config()
    drive = RoverDrive(config)
    pressed = set()
    steering = 0.0
    speed = 0.0
    max_command_speed = min(float(args.max_command_speed), float(config["rover"]["max_speed"]))
    print(
        f"manual drive: step_speed={args.step_speed:.3f}, "
        f"step_steer={args.step_steer:.3f}, max_command_speed={max_command_speed:.3f}",
    )

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
            if "w" in pressed:
                speed += args.step_speed
            elif "s" in pressed:
                speed -= args.step_speed
            else:
                speed *= SPEED_DECAY

            if "a" in pressed:
                steering -= args.step_steer
            elif "d" in pressed:
                steering += args.step_steer
            else:
                steering *= STEER_DECAY

            if "space" in pressed:
                speed = 0.0
                steering = 0.0

            speed = clip(speed, max_command_speed)
            steering = clip(steering, config["rover"]["max_steer"])
            left, right = drive.send(steering, speed)
            print(f"speed={speed:.2f} steering={steering:.2f} L={left:.2f} R={right:.2f}")
            time.sleep(UPDATE_INTERVAL)
    finally:
        drive.stop()


if __name__ == "__main__":
    main()
