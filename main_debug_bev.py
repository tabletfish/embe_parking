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
    detect_vertical_tape_boundaries,
    draw_locked_parking_slot,
    detect_slot_candidates,
    draw_parking_slots,
    draw_slots,
    draw_tape_boundaries,
    infer_parking_slots_from_mask,
)
from auto_parking.perception.tape import tape_mask  # noqa: E402


def parse_source(value):
    if value is None:
        return None
    try:
        return int(value)
    except ValueError:
        return value


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("source", nargs="?", help="camera index, video path, or omitted for CSI cam0")
    parser.add_argument("--config", default=str(PROJECT_ROOT / "config.yaml"))
    args = parser.parse_args()

    config = load_config(args.config)
    cap = open_video_source(parse_source(args.source), config)
    bev = BirdEyeView(config)
    locked_slot = None
    locked_at = 0.0
    lock_timeout_s = 3.0

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        top = bev.warp(frame)
        mask = tape_mask(top, config)
        slots = detect_slot_candidates(mask, config)
        boundaries = detect_vertical_tape_boundaries(mask, config)
        parking_slots = infer_parking_slots_from_mask(mask, config)
        now = time.monotonic()
        if parking_slots:
            locked_slot = parking_slots[0]
            locked_at = now
        elif locked_slot is not None and now - locked_at > lock_timeout_s:
            locked_slot = None

        debug = bev.draw_grid(top)
        debug = draw_slots(debug, slots)
        debug = draw_tape_boundaries(debug, boundaries)
        debug = draw_parking_slots(debug, parking_slots)
        if not parking_slots and locked_slot is not None:
            debug = draw_locked_parking_slot(debug, locked_slot.center_px, locked_slot.entry_px)

        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        if frame.shape[:2] != top.shape[:2]:
            frame_show = cv2.resize(frame, (top.shape[1], top.shape[0]))
        else:
            frame_show = frame
        combined = np.hstack([frame_show, debug, mask_bgr])
        cv2.imshow("front | BEV slots | tape mask", combined)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
