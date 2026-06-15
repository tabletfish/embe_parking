#!/usr/bin/env python3
"""
Manual rover drive.

Controls:
  w/s       : forward / reverse
  i/k       : slow forward / slow reverse
  a/d       : left / right steering
  Space     : stop immediately
  q / ESC   : quit
"""

import sys
import time
from pathlib import Path

import cv2
from pynput import keyboard as kb


PROJECT_ROOT = Path(__file__).resolve().parent
ROVER_DIR = PROJECT_ROOT / "rover"
sys.path.insert(0, str(ROVER_DIR))


SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200

MAX_SPEED = 0.5
MAX_STEER = 1.0
SPEED_STEP = 0.05
SLOW_SPEED_STEP = 0.02
STEER_STEP = 0.15
SPEED_DECAY = 0.50
STEER_DECAY = 0.60

CSI_PIPELINE = (
    "nvarguscamerasrc sensor-id=0 ! "
    "video/x-raw(memory:NVMM), width=640, height=360, framerate=30/1 ! "
    "nvvidconv ! video/x-raw, format=BGRx ! videoconvert ! appsink"
)


def _clip(value, limit):
    return max(-limit, min(limit, value))


def compute_wheel_speeds(steering, speed):
    steer = _clip(steering, MAX_STEER)
    spd = _clip(speed, MAX_SPEED)
    base = abs(spd)

    if steer >= 0:
        left = base * (1.0 - 0.9 * steer)
        right = base
    else:
        left = base
        right = base * (1.0 + 0.9 * steer)

    left = _clip(left, MAX_SPEED)
    right = _clip(right, MAX_SPEED)
    if spd < 0:
        left, right = -left, -right

    return -left, -right


def open_camera(source):
    if source is None:
        cap = cv2.VideoCapture(CSI_PIPELINE, cv2.CAP_GSTREAMER)
    else:
        cap = cv2.VideoCapture(source, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc("M", "J", "P", "G"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)
        cap.set(cv2.CAP_PROP_FPS, 30)

    if not cap.isOpened():
        raise RuntimeError(f"camera open failed: {source}")
    return cap


def connect_rover():
    try:
        from base_ctrl import BaseController

        base = BaseController(SERIAL_PORT, BAUD_RATE)
        print(f"[MANUAL] rover connected: {SERIAL_PORT}")
        time.sleep(1.0)
        return base, False
    except Exception as exc:
        print(f"[MANUAL] warning: rover connection failed ({exc}) - visualization only")
        return None, True


def main(source=None):
    cap = open_camera(source)
    base, sim_mode = connect_rover()

    speed = 0.0
    steering = 0.0
    running = True
    pressed = set()

    def stop_motors():
        if base is not None:
            base.base_json_ctrl({"T": 1, "L": 0.0, "R": 0.0})

    def send(steer, spd):
        if not sim_mode and base is not None:
            left, right = compute_wheel_speeds(steer, spd)
            base.base_json_ctrl({"T": 1, "L": left, "R": right})
            return left, right
        return 0.0, 0.0

    def on_press(key):
        nonlocal running, speed, steering
        try:
            pressed.add(key.char)
            if key.char == "q":
                running = False
        except AttributeError:
            if key == kb.Key.space:
                speed = 0.0
                steering = 0.0
                stop_motors()
            elif key == kb.Key.esc:
                running = False

    def on_release(key):
        try:
            pressed.discard(key.char)
        except AttributeError:
            pass

    listener = kb.Listener(on_press=on_press, on_release=on_release)
    listener.start()
    cv2.namedWindow("Manual", cv2.WINDOW_NORMAL)

    try:
        while running:
            ok, frame = cap.read()
            if not ok:
                break

            if "w" in pressed:
                speed = _clip(speed + SPEED_STEP, MAX_SPEED)
            elif "s" in pressed:
                speed = _clip(speed - SPEED_STEP, MAX_SPEED)
            elif "i" in pressed:
                speed = _clip(speed + SLOW_SPEED_STEP, MAX_SPEED)
            elif "k" in pressed:
                speed = _clip(speed - SLOW_SPEED_STEP, MAX_SPEED)
            else:
                speed *= SPEED_DECAY

            if "a" in pressed:
                steering = _clip(steering - STEER_STEP, MAX_STEER)
            elif "d" in pressed:
                steering = _clip(steering + STEER_STEP, MAX_STEER)
            else:
                steering *= STEER_DECAY

            cmd_left, cmd_right = send(steering, speed)

            height, width = frame.shape[:2]
            status = (
                f"spd:{speed:+.2f}  steer:{steering:+.2f}  "
                f"L:{cmd_left:+.2f} R:{cmd_right:+.2f}"
            )
            cv2.putText(frame, status, (10, height - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 165, 255), 1)
            cv2.putText(frame, "MANUAL", (width - 120, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)
            cv2.imshow("Manual", frame)
            cv2.waitKey(1)
    except KeyboardInterrupt:
        print("\n[MANUAL] Ctrl-C")
    finally:
        listener.stop()
        stop_motors()
        cap.release()
        cv2.destroyAllWindows()
        print("[MANUAL] stopped")


if __name__ == "__main__":
    src = int(sys.argv[1]) if len(sys.argv) > 1 else None
    main(src)
