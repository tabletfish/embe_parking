#!/usr/bin/env python3
"""
Record manual rover commands for later replay.

Controls:
  w/s       : forward / reverse
  a/d       : left / right steering
  Space     : stop immediately
  q / ESC   : save and quit
"""

import argparse
import json
import sys
import time
from pathlib import Path

import cv2


PROJECT_ROOT = Path(__file__).resolve().parent
ROVER_DIR = PROJECT_ROOT / "rover"
sys.path.insert(0, str(ROVER_DIR))


SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200

MAX_SPEED = 0.5
MAX_STEER = 1.0
SPEED_STEP = 0.05
STEER_STEP = 0.15
SPEED_DECAY = 0.85
STEER_DECAY = 0.60
DEFAULT_OUTPUT = PROJECT_ROOT / "task" / "parking_demo.json"

CSI_PIPELINE = (
    "nvarguscamerasrc sensor-id=0 ! "
    "video/x-raw(memory:NVMM), width=640, height=360, framerate=30/1 ! "
    "nvvidconv ! video/x-raw, format=BGRx ! videoconvert ! appsink"
)


def parse_args():
    parser = argparse.ArgumentParser(description="Record manual rover commands.")
    parser.add_argument("source", nargs="?", type=int, help="camera index; omitted for CSI cam0")
    parser.add_argument("-o", "--output", default=str(DEFAULT_OUTPUT), help="output JSON path")
    parser.add_argument("--dry-run", action="store_true", help="do not open rover serial port")
    parser.add_argument("--no-camera", action="store_true", help="record without opening a camera window")
    return parser.parse_args()


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


def connect_rover(dry_run):
    if dry_run:
        print("[RECORD] dry-run: rover serial port is not opened")
        return None, True
    try:
        from base_ctrl import BaseController

        base = BaseController(SERIAL_PORT, BAUD_RATE)
        print(f"[RECORD] rover connected: {SERIAL_PORT}")
        time.sleep(1.0)
        return base, False
    except Exception as exc:
        print(f"[RECORD] warning: rover connection failed ({exc}) - visualization only")
        return None, True


def main():
    args = parse_args()
    from pynput import keyboard as kb

    output_path = Path(args.output).expanduser()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    cap = None if args.no_camera else open_camera(args.source)
    base, sim_mode = connect_rover(args.dry_run)

    speed = 0.0
    steering = 0.0
    running = True
    pressed = set()
    samples = []
    start_time = time.monotonic()

    def stop_motors():
        if base is not None:
            base.base_json_ctrl({"T": 1, "L": 0.0, "R": 0.0})

    def send(steer, spd):
        if not sim_mode and base is not None:
            left, right = compute_wheel_speeds(steer, spd)
            base.base_json_ctrl({"T": 1, "L": left, "R": right})
            return left, right
        return compute_wheel_speeds(steer, spd)

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
    if cap is not None:
        cv2.namedWindow("Record", cv2.WINDOW_NORMAL)

    print("[RECORD] started")
    print("[RECORD] controls: w/s speed, a/d steering, space stop, q/ESC save and quit")

    try:
        while running:
            loop_start = time.monotonic()
            frame = None
            if cap is not None:
                ok, frame = cap.read()
                if not ok:
                    break

            if "w" in pressed:
                speed = _clip(speed + SPEED_STEP, MAX_SPEED)
            elif "s" in pressed:
                speed = _clip(speed - SPEED_STEP, MAX_SPEED)
            else:
                speed *= SPEED_DECAY

            if "a" in pressed:
                steering = _clip(steering - STEER_STEP, MAX_STEER)
            elif "d" in pressed:
                steering = _clip(steering + STEER_STEP, MAX_STEER)
            else:
                steering *= STEER_DECAY

            cmd_left, cmd_right = send(steering, speed)
            t = loop_start - start_time
            samples.append(
                {
                    "t": round(t, 4),
                    "steering": round(steering, 4),
                    "speed": round(speed, 4),
                    "left": round(cmd_left, 4),
                    "right": round(cmd_right, 4),
                    "keys": sorted(pressed),
                }
            )

            status = (
                f"REC t:{t:5.1f}  spd:{speed:+.2f}  steer:{steering:+.2f}  "
                f"L:{cmd_left:+.2f} R:{cmd_right:+.2f}"
            )
            print(status)

            if frame is not None:
                height, width = frame.shape[:2]
                cv2.putText(frame, status, (10, height - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 165, 255), 1)
                cv2.putText(frame, "RECORD", (width - 120, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)
                cv2.imshow("Record", frame)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break

            sleep_time = max(0.0, 0.1 - (time.monotonic() - loop_start))
            time.sleep(sleep_time)
    except KeyboardInterrupt:
        print("\n[RECORD] Ctrl-C")
    finally:
        listener.stop()
        stop_motors()
        if cap is not None:
            cap.release()
            cv2.destroyAllWindows()

    data = {
        "version": 1,
        "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "interval": 0.1,
        "step_speed": SPEED_STEP,
        "step_steer": STEER_STEP,
        "max_speed": MAX_SPEED,
        "max_steer": MAX_STEER,
        "duration": round(samples[-1]["t"], 4) if samples else 0.0,
        "samples": samples,
    }
    with output_path.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
        f.write("\n")

    print(f"[RECORD] saved {len(samples)} samples to {output_path}")


if __name__ == "__main__":
    main()
