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
    draw_parking_slots,
    draw_slots,
    draw_tape_boundaries,
    infer_parking_slots_from_mask,
)
from auto_parking.perception.tape import tape_mask  # noqa: E402
from auto_parking.planning.pure_pursuit import pure_pursuit_steering  # noqa: E402


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


def draw_control_debug(image, entry_px, target, steering, drive_enabled):
    out = image.copy()
    status = "DRIVE" if drive_enabled else "DRY-RUN"
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
    args = parser.parse_args()

    config = load_config(None if args.config == str(PROJECT_ROOT / "config.yaml") else args.config)
    cap = open_video_source(parse_source(args.source), config)
    bev = BirdEyeView(config)

    drive = None
    if args.drive:
        from auto_parking.control.drive import RoverDrive  # noqa: WPS433

        drive = RoverDrive(config)

    speed = float(args.speed if args.speed is not None else config["rover"]["default_speed"])
    wheelbase_m = float(config["rover"]["wheelbase_m"])
    last_print = 0.0

    try:
        while True:
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
            target = None
            if parking_slots:
                selected = parking_slots[0]
                target = entry_to_path_point(bev, selected.entry_px)
                path = np.array([target], dtype=float)
                steering, _ = pure_pursuit_steering(path, (0.0, 0.0, 0.0), 0.05, wheelbase_m)
                debug = draw_control_debug(debug, selected.entry_px, target, steering, args.drive)

                now = time.monotonic()
                if now - last_print >= 0.5:
                    print(
                        f"entry_px={selected.entry_px} "
                        f"target_forward={target[0]:.3f}m "
                        f"target_lateral={target[1]:+.3f}m "
                        f"steering={steering:+.3f} speed={speed:.2f}"
                    )
                    last_print = now

                if drive is not None:
                    drive.send(steering, speed)
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
