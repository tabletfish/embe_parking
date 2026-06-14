#!/usr/bin/env python3
import argparse
import sys
from pathlib import Path

import cv2

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from auto_parking.camera.csi import open_video_source  # noqa: E402
from auto_parking.config import load_config  # noqa: E402


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
    ok, frame = cap.read()
    cap.release()
    if not ok:
        raise RuntimeError("Could not read a frame")

    points = []
    win = "click BEV src points: front-left, front-right, rear-right, rear-left"

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(points) < 4:
            points.append((x, y))

    cv2.namedWindow(win)
    cv2.setMouseCallback(win, on_mouse)

    while True:
        vis = frame.copy()
        for idx, point in enumerate(points):
            cv2.circle(vis, point, 5, (0, 0, 255), -1)
            cv2.putText(vis, str(idx + 1), (point[0] + 8, point[1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.imshow(win, vis)
        key = cv2.waitKey(20) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == ord("r"):
            points.clear()
        if len(points) == 4:
            print("src_points:")
            for x, y in points:
                print(f"  - [{x}, {y}]")
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
