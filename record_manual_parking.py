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
DEFAULT_COMMAND_MAX_SPEED = 0.5
SPEED_DECAY = 0.85
STEER_DECAY = 0.60
DEFAULT_OUTPUT = PROJECT_ROOT / "recordings" / "parking_demo.json"


def parse_args():
    parser = argparse.ArgumentParser(
        description="Record manual rover commands for later replay.",
    )
    parser.add_argument(
        "source",
        nargs="?",
        help="camera index, video path, or omitted for CSI cam0",
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
        help="recording speed limit",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="record commands without opening the rover serial port",
    )
    parser.add_argument(
        "--no-camera",
        action="store_true",
        help="record without opening the BEV camera window",
    )
    return parser.parse_args()


def parse_source(value):
    if value is None:
        return None
    try:
        return int(value)
    except ValueError:
        return value


def key_name(key):
    from pynput import keyboard

    try:
        return key.char
    except AttributeError:
        if key == keyboard.Key.space:
            return "space"
    return None


class BevRecorderView:
    def __init__(self, config, source):
        import cv2

        from auto_parking.camera.csi import open_video_source
        from auto_parking.perception.bev import BirdEyeView

        self.cv2 = cv2
        self.cap = open_video_source(parse_source(source), config)
        self.bev = BirdEyeView(config)
        self.config = config
        self.locked_slot = None
        self.locked_at = 0.0
        self.lock_timeout_s = 3.0
        cv2.namedWindow("record | front | BEV slots | tape mask", cv2.WINDOW_NORMAL)

    def update(self, state_text):
        import numpy as np

        from auto_parking.perception.slot_detector import (
            detect_slot_candidates,
            detect_vertical_tape_boundaries,
            draw_locked_parking_slot,
            draw_parking_slots,
            draw_slots,
            draw_tape_boundaries,
            infer_parking_slots_from_mask,
        )
        from auto_parking.perception.tape import tape_mask

        ok, frame = self.cap.read()
        if not ok:
            return True

        top = self.bev.warp(frame)
        mask = tape_mask(top, self.config)
        slots = detect_slot_candidates(mask, self.config)
        boundaries = detect_vertical_tape_boundaries(mask, self.config)
        parking_slots = infer_parking_slots_from_mask(mask, self.config)
        now = time.monotonic()
        if parking_slots:
            self.locked_slot = parking_slots[0]
            self.locked_at = now
        elif self.locked_slot is not None and now - self.locked_at > self.lock_timeout_s:
            self.locked_slot = None

        debug = self.bev.draw_grid(top)
        debug = draw_slots(debug, slots)
        debug = draw_tape_boundaries(debug, boundaries)
        debug = draw_parking_slots(debug, parking_slots)
        if not parking_slots and self.locked_slot is not None:
            debug = draw_locked_parking_slot(debug, self.locked_slot.center_px, self.locked_slot.entry_px)

        mask_bgr = self.cv2.cvtColor(mask, self.cv2.COLOR_GRAY2BGR)
        if frame.shape[:2] != top.shape[:2]:
            frame_show = self.cv2.resize(frame, (top.shape[1], top.shape[0]))
        else:
            frame_show = frame

        combined = np.hstack([frame_show, debug, mask_bgr])
        self.cv2.putText(
            combined,
            state_text,
            (16, 28),
            self.cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 255),
            2,
            self.cv2.LINE_AA,
        )
        self.cv2.imshow("record | front | BEV slots | tape mask", combined)
        key = self.cv2.waitKey(1) & 0xFF
        return key not in (ord("q"), 27)

    def close(self):
        self.cap.release()
        self.cv2.destroyWindow("record | front | BEV slots | tape mask")


def main():
    args = parse_args()
    from pynput import keyboard

    config = load_config(args.config)
    output_path = Path(args.output).expanduser()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    max_command_speed = min(float(args.max_command_speed), float(config["rover"]["max_speed"]))
    pressed = set()
    steering = 0.0
    speed = 0.0
    samples = []
    start_time = time.monotonic()
    drive = None
    bev_view = None
    listener = None

    print("Manual parking record started.")
    print("Controls: w/s speed, a/d steering, space stop, q finish and save.")
    print(
        f"Input steps: speed={args.step_speed:.3f}, steering={args.step_steer:.3f}, "
        f"max_command_speed={max_command_speed:.3f}",
    )
    if args.dry_run:
        print("DRY-RUN: serial port is not opened.")
    if args.no_camera:
        print("Camera view disabled.")
    else:
        print("Camera view enabled: front | BEV slots | tape mask.")

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

    try:
        bev_view = None if args.no_camera else BevRecorderView(config, args.source)
        drive = None if args.dry_run else RoverDrive(config)
        listener = keyboard.Listener(on_press=on_press, on_release=on_release)
        listener.start()

        while listener.running:
            loop_start = time.monotonic()

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

            max_speed = float(config["rover"]["max_speed"])
            max_steer = float(config["rover"]["max_steer"])
            speed = clip(speed, max_command_speed)
            steering = clip(steering, max_steer)
            turn_gain = float(config["rover"].get("turn_gain", 0.9))
            left, right = compute_wheel_speeds(steering, speed, max_steer, max_speed, turn_gain)
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
            if bev_view is not None:
                keep_running = bev_view.update(
                    f"REC t={t:5.1f}s speed={speed:+.2f} steer={steering:+.2f} "
                    f"L={left:+.2f} R={right:+.2f}",
                )
                if not keep_running:
                    break

            sleep_time = max(0.0, args.interval - (time.monotonic() - loop_start))
            time.sleep(sleep_time)
    finally:
        if drive is not None:
            drive.stop()
        if bev_view is not None:
            bev_view.close()
        if listener is not None:
            listener.stop()

    data = {
        "version": 1,
        "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "interval": args.interval,
        "step_speed": args.step_speed,
        "step_steer": args.step_steer,
        "max_command_speed": max_command_speed,
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
