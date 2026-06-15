#!/usr/bin/env python3
import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from auto_parking.camera.csi import open_video_source  # noqa: E402
from auto_parking.config import load_config  # noqa: E402
from auto_parking.perception.bev import BirdEyeView  # noqa: E402
from auto_parking.perception.slot_detector import (  # noqa: E402
    detect_slot_candidates,
    detect_vertical_tape_boundaries,
    draw_locked_parking_slot,
    draw_parking_slots,
    draw_slots,
    draw_tape_boundaries,
    infer_parking_slots_from_mask,
)
from auto_parking.perception.tape import tape_mask  # noqa: E402
from auto_parking.planning.pure_pursuit import pure_pursuit_steering  # noqa: E402
from auto_parking.control.drive import compute_wheel_speeds  # noqa: E402


def parse_source(value):
    if value is None:
        return None
    try:
        return int(value)
    except ValueError:
        return value


def entry_to_path_point(bev, entry_px):
    lateral_m, forward_m = bev.pixel_to_vehicle(entry_px)
    return forward_m, lateral_m


def draw_control_debug(image, entry_px, target, steering, drive_enabled, locked=False, reached=False):
    out = image.copy()
    status = "DRIVE" if drive_enabled else "DRY-RUN"
    if reached:
        status = "AT_ENTRY"
    elif locked:
        status += " LOCKED"
    cv2.putText(
        out,
        f"{status} steer={steering:+.2f} target=({target[0]:.2f}m,{target[1]:+.2f}m)",
        (16, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.arrowedLine(
        out,
        (out.shape[1] // 2, out.shape[0] - 1),
        entry_px,
        (0, 255, 255),
        2,
        tipLength=0.08,
    )
    return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("source", nargs="?", help="camera index, video path, or omitted for CSI cam0")
    parser.add_argument("--config", default=str(PROJECT_ROOT / "config.yaml"))
    parser.add_argument("--drive", action="store_true", help="send low-speed commands to the rover")
    parser.add_argument("--speed", type=float, default=None, help="override rover.default_speed")
    parser.add_argument("--entry-threshold", type=float, default=0.08, help="stop when entry is this close in meters")
    args = parser.parse_args()

    config = load_config(None if args.config == str(PROJECT_ROOT / "config.yaml") else args.config)
    cap = open_video_source(parse_source(args.source), config)
    bev = BirdEyeView(config)

    drive = None
    if args.drive:
        from auto_parking.control.drive import RoverDrive  # noqa: WPS433

        drive = RoverDrive(config)

    speed = float(args.speed if args.speed is not None else config["rover"]["default_speed"])
    speed = min(speed, float(config["rover"]["max_speed"]))
    wheelbase_m = float(config["rover"]["wheelbase_m"])
    last_print = 0.0
    last_loop = time.monotonic()
    locked_target = None
    locked_entry_px = None
    locked_center_px = None

    try:
        while True:
            now = time.monotonic()
            dt = now - last_loop
            last_loop = now

            ok, frame = cap.read()
            if not ok:
                break

            top = bev.warp(frame)
            mask = tape_mask(top, config)
            slots = detect_slot_candidates(mask, config)
            boundaries = detect_vertical_tape_boundaries(mask, config)
            parking_slots = infer_parking_slots_from_mask(mask, config)

            debug = bev.draw_grid(top)
            debug = draw_slots(debug, slots)
            debug = draw_tape_boundaries(debug, boundaries)
            debug = draw_parking_slots(debug, parking_slots)

            steering = 0.0
            target = locked_target
            entry_px = locked_entry_px
            target_locked = target is not None
            if parking_slots:
                selected = parking_slots[0]
                target = entry_to_path_point(bev, selected.entry_px)
                entry_px = selected.entry_px
                locked_target = target
                locked_entry_px = entry_px
                locked_center_px = selected.center_px
                target_locked = False

            if locked_target is not None and not parking_slots:
                remaining_forward = max(0.0, locked_target[0] - speed * dt)
                locked_target = (remaining_forward, locked_target[1])
                target = locked_target
                if locked_center_px is not None and locked_entry_px is not None:
                    debug = draw_locked_parking_slot(debug, locked_center_px, locked_entry_px)

            if target is not None:
                at_entry = target[0] <= args.entry_threshold
                path = np.array([target], dtype=float)
                if at_entry:
                    steering = 0.0
                    command_speed = 0.0
                else:
                    steering, _ = pure_pursuit_steering(path, (0.0, 0.0, 0.0), 0.05, wheelbase_m)
                    command_speed = speed
                left_cmd, right_cmd = compute_wheel_speeds(
                    steering,
                    command_speed,
                    float(config["rover"]["max_steer"]),
                    float(config["rover"]["max_speed"]),
                )
                if entry_px is not None:
                    debug = draw_control_debug(debug, entry_px, target, steering, args.drive, target_locked, at_entry)

                if now - last_print >= 0.5:
                    print(
                        f"mode={'DRIVE' if args.drive else 'DRY-RUN'} "
                        f"{'LOCKED ' if target_locked else ''}"
                        f"{'AT_ENTRY ' if at_entry else ''}"
                        f"entry_px={entry_px} "
                        f"target_forward={target[0]:.3f}m "
                        f"target_lateral={target[1]:+.3f}m "
                        f"steering={steering:+.3f} speed={command_speed:.2f} "
                        f"L={left_cmd:+.3f} R={right_cmd:+.3f}"
                    )
                    last_print = now

                if drive is not None:
                    if at_entry:
                        drive.stop()
                    else:
                        drive.send(steering, command_speed)
            elif drive is not None:
                drive.stop()

            mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            if frame.shape[:2] != top.shape[:2]:
                frame_show = cv2.resize(frame, (top.shape[1], top.shape[0]))
            else:
                frame_show = frame
            combined = np.hstack([frame_show, debug, mask_bgr])
            cv2.imshow("front | level1 parking | tape mask", combined)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
    finally:
        if drive is not None:
            drive.stop()
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
