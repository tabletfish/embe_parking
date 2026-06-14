#!/usr/bin/env python3
import sys
import time
from pathlib import Path

from pynput import keyboard

PROJECT_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from auto_parking.config import load_config  # noqa: E402
from auto_parking.control.drive import RoverDrive, clip  # noqa: E402


STEP_STEER = 0.15
STEP_SPEED = 0.05
UPDATE_INTERVAL = 0.1


def main():
    config = load_config()
    drive = RoverDrive(config)
    pressed = set()
    steering = 0.0
    speed = 0.0

    def on_press(key):
        try:
            pressed.add(key.char)
        except AttributeError:
            if key == keyboard.Key.space:
                pressed.add("space")

    def on_release(key):
        try:
            pressed.discard(key.char)
            if key.char == "q":
                return False
        except AttributeError:
            if key == keyboard.Key.space:
                pressed.discard("space")

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    try:
        while listener.running:
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

            speed = clip(speed, config["rover"]["max_speed"])
            steering = clip(steering, config["rover"]["max_steer"])
            left, right = drive.send(steering, speed)
            print(f"speed={speed:.2f} steering={steering:.2f} L={left:.2f} R={right:.2f}")
            time.sleep(UPDATE_INTERVAL)
    finally:
        drive.stop()


if __name__ == "__main__":
    main()

